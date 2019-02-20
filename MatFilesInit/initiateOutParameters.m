function outParams = initiateOutParameters(simParams,fileCfg,varargin)
% function outParams = initiateOutParameters(fileCfg,varargin)
%
% Settings of the outputs
% It takes in input the name of the (possible) file config and the inputs
% of the main function
% It returns the structure "outParams"

fprintf('Output settings\n');

% [outputFolder]
% Folder where the output files are recorded
% If the folder is not present, the simulator creates it
outParams = addNewParam([],'outputFolder','Output','Folder for the output files','string',fileCfg,varargin{1}{1});
outParams.outputFolder = sprintf('%s/%s',pwd,outParams.outputFolder);
fprintf('Full path of the output folder = %s\n',outParams.outputFolder);
if exist(outParams.outputFolder)~=7
    mkdir(outParams.outputFolder);
end

% Name of the file that summarizes the inputs and outputs of the simulation
% Each simulation adds a line in append
% The file is a normal text file
% The name of the file cannot be changed
outParams.outMainFile = 'MainOut.xls';
fprintf('Main output file = %s/%s\n',outParams.outputFolder,outParams.outMainFile);

% Simulation ID
mainFileName = sprintf('%s/%s',outParams.outputFolder,outParams.outMainFile);
fid = fopen(mainFileName);
if fid==-1
    simID = 0;
else
    fclose(fid);
    C = textread(mainFileName, '%s','delimiter', '\n');
    lastLine = C{end};
    for i=1:length(lastLine)
        if lastLine(i)=='v'
            simID = str2num(lastLine(1:i-1));
            break;
        end
    end
end
outParams.simID = simID+1;
fprintf('Simulation ID = %.0f\n',outParams.simID);

% [printNeighbors]
% Boolean to activate the print to file of the number of neighbors
outParams = addNewParam(outParams,'printNeighbors',false,'Activate the print to file of the number of neighbors','bool',fileCfg,varargin{1}{1});

% [printUpdateDelay]
% Boolean to activate the print to file of the update delay between received beacons
outParams = addNewParam(outParams,'printUpdateDelay',false,'Activate the print to file of the update delay between received beacons','bool',fileCfg,varargin{1}{1});

% [printPacketDelay]
% Boolean to activate the print to file of the packet delay between received beacons
outParams = addNewParam(outParams,'printPacketDelay',false,'Activate the print to file of the packet delay between received beacons','bool',fileCfg,varargin{1}{1});

% [delayResolution]
% Delay resolution (s)
if outParams.printUpdateDelay || outParams.printPacketDelay
    outParams = addNewParam(outParams,'delayResolution',0.001,'Delay resolution (s)','double',fileCfg,varargin{1}{1});
    if outParams.delayResolution<=0
        error('Error: "outParams.delayResolution" cannot be <= 0');
    end
end

% [printDistanceDetails]
% Boolean to activate the print to file of the details for distances from 0
% up to the maximum awareness range
outParams = addNewParam(outParams,'printDistanceDetails',false,'Activate the print to file of the details for distances from 0 up to the maximum awareness range','bool',fileCfg,varargin{1}{1});

% [printPRRmap]
% Boolean to activate the creation and print of a PRR map (only for urban scenarios)
outParams = addNewParam(outParams,'printPRRmap',false,'Activate the creation and print of a PRR map (only for urban scenarios)','bool',fileCfg,varargin{1}{1});
% Check if using fileObstaclesMap
if ~simParams.fileObstaclesMap
    outParams.printPRRmap = false;
end

% [printPowerControl]
% Boolean to activate the print to file of the power control allocation
outParams = addNewParam(outParams,'printPowerControl',false,'Activate the print to file of the power control allocation','bool',fileCfg,varargin{1}{1});

% [powerResolution]
% Power resolution (dBm)
if outParams.printPowerControl
    outParams = addNewParam(outParams,'powerResolution',1,'Power resolution (dbm)','double',fileCfg,varargin{1}{1});
    if outParams.powerResolution<=0
        error('Error: "outParams.powerResolution" cannot be <= 0');
    end
end

fprintf('\n');
%
%%%%%%%%%
