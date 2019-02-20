function appParams = initiateApplicationParameters(fileCfg,varargin)
% function appParams = initiateApplicationParameters(fileCfg,varargin)
%
% Settings of the application
% It takes in input the name of the (possible) file config and the inputs
% of the main function
% It returns the structure "appParams"

fprintf('Application settings\n');

% [Tbeacon]
% Beacon period in seconds.
appParams = addNewParam([],'Tbeacon',0.1,'Beacon period (s)','double',fileCfg,varargin{1}{1});
if appParams.Tbeacon<=0
    error('Error: "appParams.Tbeacon" cannot be <= 0');
end
% The beacon periodicity fB is derived
appParams.fB = 1/appParams.Tbeacon;

% [beaconSize]
% Beacon size (Bytes)
appParams = addNewParam(appParams,'beaconSizeBytes',190,'Beacon size (Bytes)','integer',fileCfg,varargin{1}{1});
if appParams.beaconSizeBytes<=0 || appParams.beaconSizeBytes>10000
    error('Error in the setting of "appParams.beaconSizeBytes".');
end

% [resourcesV2V]
% Resource allocated to V2V (%)
appParams = addNewParam(appParams,'resourcesV2V',100,'Resource allocated to V2V (%)','integer',fileCfg,varargin{1}{1});
if appParams.resourcesV2V<=0 || appParams.resourcesV2V>100
    error('Error in the setting of "appParams.resourcesV2V". Not within 1-100%.');
end


fprintf('\n');
% 
%%%%%%%%%
