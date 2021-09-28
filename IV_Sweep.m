function [timestamp,voltage,currentDensity,pdCurrent] = IV_Sweep
%% MEASUREMENT VARIABLES -- EDIT THESE

% Script automatically records following variables in filename: nameSample,
% scanRate, beginVoltage, endpointVoltage,sweeps

% All variables (including delay, NPLC, currentLimit, etc.) are also
% written out to csv file along with data

sweeps = 2;           % beginVoltage -> endpointVoltage == 1 SWEEP
beginVoltage = -0.5;
endpointVoltage = 5.5;
currentLimit = 0.1;  % A, limit for current when sourcing voltage

scanRate = 300;  % mV/second
stepSizemV = 100; % mV

delay = 0; % seconds delay between change voltage, measure current
NPLC = 1; % 1 = 1/60sec, 0.01=fast, 1=normal, 6=slow, 10=slowest

sampleArea = 0.1; % cm^2
nameSample = 'xxx';

%% CALCULATED VARIABLES

if beginVoltage > endpointVoltage
    stepSizemV = -stepSizemV;
end

voltageStepSize = stepSizemV / 1000;

pAmmeterIntTime = 0.15; % measurement time for picoammeter
period = abs(voltageStepSize) / scanRate * 1000;
measurementOverhead = period - pAmmeterIntTime - (delay + NPLC) / 60;

if measurementOverhead < 0.05
    error('Your added delay is too long, or NPLC too large.')
end

%% BUILD VOLTAGE SOURCING VECTOR

% build each section of one cycle
beginToEndpoint = transpose(beginVoltage+voltageStepSize:voltageStepSize:endpointVoltage);
endpointToBegin = transpose(endpointVoltage-voltageStepSize:-voltageStepSize:beginVoltage);

trackingvoltage = zeros(0,1);

for i = 1:sweeps
    if mod(i,2) == 1
        trackingvoltage = [trackingvoltage;beginToEndpoint];
    else
        trackingvoltage = [trackingvoltage;endpointToBegin];
    end
end

% add start point in measurement
trackingvoltage = [beginVoltage;trackingvoltage];

% round voltage to 5 decimal points, floating point error
trackingvoltage = round(trackingvoltage,5);

totalMeasurements = length(trackingvoltage);

%% INITIALIZE MEASUREMENT VECTORS

timestamp = zeros(totalMeasurements,1);
voltage   = zeros(totalMeasurements,1);
current   = zeros(totalMeasurements,1);
pdCurrent = zeros(totalMeasurements,1);

%% INITIALIZE INSTRUMENTS

% Find a VISA-GPIB object.
k2400 = instrfind('Type', 'visa-gpib', 'RsrcName', 'GPIB0::24::INSTR', 'Tag', '');

% Create the VISA-GPIB object if it does not exist
% otherwise use the object that was found.
if isempty(k2400)
    k2400 = visa('NI', 'GPIB0::24::INSTR', 'InputBufferSize', 2500);
else
    fclose(k2400);
    k2400 = k2400(1);
end

% Connect to instrument object
fopen(k2400);

% reset instrument
fprintf(k2400, '*RST');

% rear measurement
fprintf(k2400,':ROUT:TERM FRONT');

% set K2400 to 4 wire sense
fprintf(k2400,':SYSTEM:RSENSE ON');

% set K2400 to source voltage
fprintf(k2400,':SOUR:FUNC VOLT');
fprintf(k2400,':SOUR:VOLT:MODE FIXED');
fprintf(k2400,':SOUR:VOLT:RANG 21');
fprintf(k2400,':SOURCE:VOLT:LEV 0');
fprintf(k2400,[':SENS:CURR:PROT ',num2str(currentLimit)]);

% set K2400 to measure current
fprintf(k2400,':SENS:FUNC "CURR"');
fprintf(k2400,':SENS:CURR:RANG:AUTO ON');
fprintf(k2400,[':SENS:CURR:NPLC ',num2str(NPLC)]);
fprintf(k2400,':FORM:ELEM CURR');

% turn on k2400 output
fprintf(k2400,':OUTP ON');

%% INITIALIZE INSTRUMENTS - Picoammeter

% Find a VISA-GPIB object.
hp4140b = instrfind('Type', 'visa-gpib', 'RsrcName', 'GPIB0::17::INSTR', 'Tag', '');

% Create the VISA-GPIB object if it does not exist
% otherwise use the object that was found.
if isempty(hp4140b)
    hp4140b = visa('NI', 'GPIB0::17::INSTR');
else
    fclose(hp4140b);
    hp4140b = hp4140b(1);
end

% Connect to instrument object, obj1.
fopen(hp4140b);

% measure current
fprintf(hp4140b,'F1');

% auto range
fprintf(hp4140b,'RA1');

% shortest measurement integration time
fprintf(hp4140b,'I1');

% manual trigger measurement
fprintf(hp4140b,'T3');

%% TIMER

% color changing with time for each point in sweep
c = linspace(1,10,length(voltage));

figure('Name',['I-V: ',nameSample,', ',num2str(scanRate),' mV/sec, ',num2str(sweeps),' sweep']);
ylabel('Current (A)');
xlabel('Voltage (V)');
scatterPlot = scatter(voltage,current,[],c,'filled');
set(scatterPlot,'XDataSource','voltage');
set(scatterPlot,'YDataSource','current');

% measurement variables
count = 0;
tic;

% initialize and start the timer, which controls measurements
t = timer('TimerFcn',@measure,'Period',period,'TasksToExecute',...
    totalMeasurements,'ExecutionMode','fixedRate');

% pause before starting timer to give time for instruments to initialize
pause(2);

% start timer
start(t);

    function measure(~,~)
        count = count + 1;
        
        % set the new voltage
        fprintf(k2400,[':SOURCE:VOLT:LEV ',num2str(trackingvoltage(count))]);
        
        % delay between changing voltage and measuring current
        pause(delay);
        
        % measure current
        current(count) = str2double(query(k2400,'READ?'));
        
        % measure pdCurrent
        a = query(hp4140b,'E');
        pdCurrent(count) = str2double(a(4:13));
        
        % store timestamp and voltage
        timestamp(count) = toc;
        voltage(count) = trackingvoltage(count);
        
        % display the voltage and current in Command Window
        fprintf('Voltage: ');
        fprintf('%0.3f',trackingvoltage(count));
        fprintf('   Current:  ');
        fprintf('%10.9e',current(count));
        fprintf('   PD Current:  ');
        fprintf('%10.9e',pdCurrent(count));
        disp(' ');
        
        % live update graph if enough time between measurements
        if measurementOverhead > 0.5
            assignin('base','voltage',voltage);
            assignin('base','current',current);
            assignin('base','pdCurrent',pdCurrent);
            refreshdata;
        end
    end

%% DATA PROCESSING

% wait to return data until timer finished
wait(t);

% start timestamp at 0 seconds for first measurement
timestamp = timestamp - timestamp(1);

% calculate current density (mA/cm^2)
currentDensity = current * 1000 / sampleArea;

% calculate actual average scan rate, in mV
averageScanRate = sweeps*abs(beginVoltage - endpointVoltage) / timestamp(end) * 1000;

% send variables to workspace
assignin('base','timestamp',timestamp);
assignin('base','voltage',voltage);
assignin('base','current',current);
assignin('base','pdCurrent',pdCurrent);
assignin('base','currentDensity',currentDensity);
assignin('base','trackingvoltage',trackingvoltage);

% plot I-V curves

% if the linear plot that live updates did not update because of too little
% overhead, refresh here to make sure the graph is updated
refreshdata;

% plot photodiode current curve
figure('Name',['Photo Diode: ',nameSample,', ',num2str(scanRate),' mV/sec, ',num2str(sweeps),' sweep']);
ylabel('Current (A)');
xlabel('Time (s)');
scatter(timestamp,pdCurrent,[],c,'filled');

% export data to csv
nameScanRate = [num2str(scanRate),'mVperSec'];
nameSweeps = [num2str(sweeps),'sweeps'];
nameVoltageRange = [num2str(endpointVoltage),'to',num2str(beginVoltage),'V'];

data = num2cell([timestamp,voltage,currentDensity,pdCurrent]);

info = cell(totalMeasurements,1);
info{1} = ['Sweeps = ',num2str(sweeps)];
info{2} = ['Begin Voltage = ',num2str(beginVoltage)];
info{3} = ['Endpoint Voltage = ',num2str(endpointVoltage)];
info{4} = ['Requested Scan Rate = ',num2str(scanRate),' mV/sec'];
info{5} = ['Actual Scan Rate = ',num2str(averageScanRate),' mV/sec'];
info{6} = ['Step Size = ',num2str(stepSizemV),' mV/step'];
info{7} = ['Delay Between Source & Measure = ',num2str(delay),' sec'];
info{8} = ['NPLC = ',num2str(NPLC),' = ',num2str(NPLC/60),' sec integration time'];
info{9} = ['Current Sourcing Limit = ',num2str(currentLimit),' A'];

c = [data,info];

% write to csv
T = cell2table(c,'VariableNames',{'Timestamp','Voltage_V',...
    'Current_mAcm2','PD_current_A','Info'});
writetable(T,['IV','_',nameSample,'_',nameScanRate,'_',...
    nameVoltageRange,'_',nameSweeps,'.csv']);

% turn off Keithley 2400 output
fprintf(k2400,':OUTP OFF');

end