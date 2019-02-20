function [simParams,appParams,phyParams,outParams] = initiateParameters(varargin)
% Function to initialize simulator parameters and 

nArgs = length(varargin{1});

%%%%%%%%%
% Check file config and read parameters
if nArgs < 1 || strcmp(varargin{1}{1},'0') || strcmp(varargin{1}{1},'default')
    fileCfg = 'LTEV2Vsim.cfg';
elseif mod(nArgs-1,2)~=0
    error('Error in the number or value of input parameters. Simulation aborted.');
elseif nArgs == 1 || strcmp(varargin{1}{1},'help')
    fileCfg = '';
else
    fileCfg = char(varargin{1}{1});
end
fid = fopen(fileCfg);
if fid==-1
    if ~strcmp(varargin{1}{1},'help')
        fprintf('File config "%s" not found. Simulation continues anyway.\n\n',fileCfg);
    end
else
    fclose(fid);
end
%%%%%%%%%

% Initialize Application parameters
appParams = initiateApplicationParameters(fileCfg,varargin);

% Initialize Simulation parameters
simParams = initiateMainSimulationParameters(appParams.Tbeacon,fileCfg,varargin);
simParams.fileCfg = fileCfg;

% Initialize PHY layer parameters
phyParams = initiatePhyParameters(simParams,appParams,fileCfg,varargin);

% Initialize Output parameters
outParams = initiateOutParameters(simParams,fileCfg,varargin);
    
% LTE-V2V derived parameters
if simParams.useLTE
    
    % Initialize LTE resource assignement algorithm parameters
    [simParams,phyParams] = initiateBRAssignmentAlgorithm(simParams,phyParams,appParams.Tbeacon,fileCfg,varargin);
    
    % Derive LTE resources available for beaconing (Beacon Resources)
    [appParams,phyParams] = deriveBeaconResources(appParams,phyParams);
    
end

end
