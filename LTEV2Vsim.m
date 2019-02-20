function [outputValues,simParams,appParams,phyParams,outParams] = LTEV2Vsim(varargin)
% The function LTEV2Vsim() is the main function of the simulator

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%               LTEV2Vsim              %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Call it as
% LTEV2Vsim(fileCfg,paramName1,value1,...paramNameN,valueN)
%
% Parameters are optional.
% If one or more parameters are given in input, the first corresponds to the
% config file (a text file). Use 'default' or '0' to set the default config
% file (i.e., LTEV2Vsim.cfg). If a file that does not exist is set, the
% simulation continues without considering the settings from the config
% file.
% In the config file, write couples with i) the parameter name within squared
% brackets and ii) the value of the parameter.
%
% In the command line, couples of parameters follow the config file. Each couple
% must include i) the parameter name and ii) the value.
%
% All parameters can be set in the config file and/or in the command
% line; the priority is: 1) command line; 2) config file; 3) default value.
%
% Example call:
% LTEV2Vsim('default','seed',0,'MCS',2);
% In this example, the seed for random numbers is randomly selected and the
% MCS 2 is set. Then the other parameters take the value from the default
% config file if the file is present and the parameter is set; otherwise
% the default value is used.
%
% Write
% LTEV2Vsim('help')
% for a full list of the parameters with their default values.

%% Initialization

% Version of the simulator
simVersion = 'v3.5';
fprintf('LTEV2Vsim %s\n\n',simVersion);

% The path of the directory of the simulator is saved in 'fullPath'
fullPath = which('LTEV2Vsim');
[~,fullPathUnix] = strtok(fliplr(fullPath), '/');
[~,fullPathWindows] = strtok(fliplr(fullPath), '\');
if isempty(fullPathUnix)
    fullPath = fliplr(fullPathWindows);
else
    fullPath = fliplr(fullPathUnix);
end
addpath(genpath(fullPath));
chdir(fullPath);

% 'help' feature:
% "LTEV2Vsim('help')" allows to print the full list of parameters
% with default values
if nargin == 1 || strcmp(varargin{1},'help')
    fprintf('Help: list of the parameters with default values\n\n');
    initiateParameters({'help'});
    fprintf('End of the list.\n');
    return
end

% Simulator parameters and initial settings
[simParams,appParams,phyParams,outParams] = initiateParameters(varargin);

% Update PHY structure with the ranges
[phyParams] = deriveRanges(phyParams,simParams);

% Simulator output inizialization
outputValues = struct('computationTime',-1,'blockingRateNoBorderTOT',-1,...
    'errorRateNoBorderTOT',-1,'packetReceptionRatioTOT',-1,...
    'NvehiclesNoBorderTOT',0,'NneighborsNoBorderTOT',0,...
    'StDevNeighboursNoBorderTOT',0,'NblockedNoBorderTOT',0,...
    'NreassignNoBorderTOT',0,'NunlockedNoBorderTOT',0,...
    'NerrorsNoBorderTOT',0,'NreceivedBeaconsNoBorderTOT',0,...
    'NcorrectlyReceivedBeaconsNoBorderTOT',0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Scenario Description

% Load scenario from Trace File or generate initial positions of vehicles
[simParams,simValues] = initVehiclePositions(simParams,appParams);

% Load obstacles scenario from Obstacles Map File (if selected)
if simParams.fileObstaclesMap
    [simParams,simValues] = loadObstaclesMapFile(simParams,simValues);
else
    [simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap] = deal(-1);
end

% Initialization of matrices correctlyReceivedMap and neighborsMap (for PRRmap)
if outParams.printPRRmap
    simValues.correctlyReceivedMap = zeros(size(simValues.GridMap));
    simValues.neighborsMap = zeros(size(simValues.GridMap));
end

% Initialization of shadowing matrix
simValues.Shadowing_dB = randn(length(simValues.IDvehicle),length(simValues.IDvehicle))*phyParams.stdDevShadowLOS_dB;
simValues.Shadowing_dB = triu(simValues.Shadowing_dB,1)+triu(simValues.Shadowing_dB)';

if outParams.printUpdateDelay
    % Initialize matrix containing update time of the received beacons
    simValues.updateTimeMatrix = -1*ones(simValues.maxID,simValues.maxID);
    
    % Initialize array with the counters of update delay events
    % (max 10 s + delayResolution -> delays larger than 10 s are
    % registered in the last element of the array)
    NupdateDelayEvents = round(10/outParams.delayResolution)+1;
    outputValues.updateDelayCounter = zeros(NupdateDelayEvents,1);
end

if outParams.printPacketDelay
    % Initialize array with the counters of packet delay events
    % (max Tbeacon/delayResolution -> delays larger than Tbeacon are
    % registered in the last element of the array)
    NpacketDelayEvents = round(appParams.Tbeacon/outParams.delayResolution);
    outputValues.packetDelayCounter = zeros(NpacketDelayEvents,1);
end

if outParams.printDistanceDetails
    % Initialize array with the counters of Rx details vs. distance (up to RawMax)
    % [distance, #Correctly decoded beacons, #Errors, #Blocked neighbors, #Neighbors (computed in printDistanceDetailsCounter)]
    outputValues.distanceDetailsCounter = zeros(floor(phyParams.RawMax),4);
    outputValues.distanceDetailsCounter(:,1) = 1:floor(phyParams.RawMax);
end

if outParams.printPowerControl
    % Initialize array with the counters of power control events
    % (max Ptx/powerResolution + 10dBm margin -> TX power higher than 
    % PtxMax + 10 dBm are registered in the last element of the array)
    % (min -100 dBm -> TX power lower than -100dBm are registered in the
    % first element of the array)
    NpowerControlEvents = round(101/outParams.powerResolution) + round(phyParams.Ptx_dBm/outParams.powerResolution);
    outputValues.powerControlCounter = zeros(NpowerControlEvents,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Start Simulation

if simParams.useLTE
    
    % LTE-V2V simulation
    [simValues,outputValues,appParams,simParams,phyParams,outParams] = mainLTEV2V(appParams,simParams,phyParams,outParams,simValues,outputValues);
    
else
    
    % 802.11p simulation
    [simValues,outputValues,appParams,simParams,phyParams,outParams] = main11p(appParams,simParams,phyParams,outParams,simValues,outputValues);
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% KPIs Computation (Output)

% Average Blocking Rate (removing border effect)
outputValues.blockingRateNoBorderTOT = outputValues.NblockedNoBorderTOT / outputValues.NvehiclesNoBorderTOT;

% Average Error Rate (removing border effect)
outputValues.errorRateNoBorderTOT = outputValues.NerrorsNoBorderTOT / outputValues.NreceivedBeaconsNoBorderTOT;

% Average Packet Reception Ratio (removing border effect)
outputValues.packetReceptionRatioTOT = outputValues.NcorrectlyReceivedBeaconsNoBorderTOT / outputValues.NneighborsNoBorderTOT;

% Average number of successful BR reassignment per vehicle per cycle (removing border effect)
outputValues.NreassignNoBorderTOT = outputValues.NreassignNoBorderTOT / outputValues.NvehiclesNoBorderTOT;

% Average number of neighbors per vehicle (removing border effect)
outputValues.NneighborsNoBorderTOT = outputValues.NneighborsNoBorderTOT / outputValues.NvehiclesNoBorderTOT;
outputValues.StDevNeighboursNoBorderTOT = outputValues.StDevNeighboursNoBorderTOT / simValues.snapshots;

% Average number of vehicles in the scenario (removing border effect)
outputValues.AvgNvehiclesNoBorder = outputValues.NvehiclesNoBorderTOT / simValues.snapshots;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Print To Files

% Print update delay to file (if enabled)
if outParams.printUpdateDelay || outParams.printPacketDelay
    printDelay(outputValues,outParams);
end

% Print details for distances up to the maximum awareness range (if enabled)
if outParams.printDistanceDetails
    printDistanceDetailsCounter(outputValues.distanceDetailsCounter,outParams);
end

% Print PRRmap to file (if enabled)
if outParams.printPRRmap && simParams.fileObstaclesMap
    printPRRmapToFile(simValues,simParams,outParams);
end

% Print power control allocation to file (if enabled)
if outParams.printPowerControl
    printPowerControl(outputValues,outParams);
end

% Print to XLS file
outputToFiles(simVersion,simParams,appParams,phyParams,outParams,outputValues);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Print To Video
fprintf('\n\nAverage blocking rate = %.5f\nAverage error rate = %.5f\nPacket reception ratio = %.5f\n',outputValues.blockingRateNoBorderTOT,outputValues.errorRateNoBorderTOT,outputValues.packetReceptionRatioTOT);
fprintf('Average number of neighbors per vehicle = %.2f +- %.2f\n',outputValues.NneighborsNoBorderTOT,outputValues.StDevNeighboursNoBorderTOT);
fprintf('Average number of vehicles in the scenario = %.0f\n\n',outputValues.AvgNvehiclesNoBorder);

fclose('all');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end