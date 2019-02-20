function simParams = initiateMainSimulationParameters(Tbeacon,fileCfg,varargin)
% function simParams = initiateMainSimulationParameters(fileCfg,varargin)
%
% Main settings of the simulation
% It takes in input the name of the (possible) file config and the inputs
% of the main function
% It returns the structure "simParams"

fprintf('Simulation settings\n');

% [seed]
% Seed for the random numbers generation
% If seed = 0, the seed is randomly selected (the selected value is saved
% in the main output file)
simParams = addNewParam([],'seed',0,'Seed for random numbers','integer',fileCfg,varargin{1}{1});
if simParams.seed == 0
    simParams.seed = randi(2^32-1,1);
    fprintf('Seed used in the simulation: %d\n',simParams.seed);
end
rng(simParams.seed);

% [simulationTime]
% Duration of the simulation in seconds
simParams = addNewParam(simParams,'simulationTime',10,'Simulation duration (s)','double',fileCfg,varargin{1}{1});
if simParams.simulationTime<=0
    error('Error: "simParams.simulationTime" cannot be <= 0');
end

% [fileTrace]
% Select if want to use Trace File: true or false
simParams = addNewParam(simParams,'fileTrace',false,'If using a file trace','bool',fileCfg,varargin{1}{1});
if simParams.fileTrace~=false && simParams.fileTrace~=true
    error('Error: "simParams.fileTrace" must be equal to false or true');
end

% [fileObstaclesMap]
% Select if want to use Obstacles Map File: true or false
simParams = addNewParam(simParams,'fileObstaclesMap',false,'If using a obstacles map file','bool',fileCfg,varargin{1}{1});
if simParams.fileObstaclesMap~=false && simParams.fileObstaclesMap~=true
    error('Error: "simParams.fileObstaclesMap" must be equal to false or true');
end

% Depending on the setting of "simParams.fileTrace", other parameters must
% be set
if simParams.fileTrace
    % [filenameTrace]
    % If the trace file is used, this selects the file
    simParams = addNewParam(simParams,'filenameTrace','null.txt','File trace name','string',fileCfg,varargin{1}{1});
    % Check that the file exists. If the file does not exist, the
    % simulation is aborted.
    fid = fopen(simParams.filenameTrace);
    if fid==-1
        fprintf('File trace "%s" not found. Simulation Aborted.',simParams.filenameTrace);
    else
        fclose(fid);
    end
    
    % [XminTrace]
    % Minimum X coordinate to keep in the traffic trace (m)
    simParams = addNewParam(simParams,'XminTrace',-1,'Minimum X coordinate to keep in the traffic trace (m)','double',fileCfg,varargin{1}{1});
    if simParams.XminTrace~=-1 && simParams.XminTrace<0
        error('Error: the value set for "appParams.XminTrace" is not valid');
    end
    
    % [XmaxTrace]
    % Maximum X coordinate to keep in the traffic trace (m)
    simParams = addNewParam(simParams,'XmaxTrace',-1,'Maximum X coordinate to keep in the traffic trace (m)','double',fileCfg,varargin{1}{1});
    if simParams.XmaxTrace~=-1 && simParams.XmaxTrace<0 && simParams.XmaxTrace<simParams.XminTrace
        error('Error: the value set for "appParams.XmaxTrace" is not valid');
    end
    
    % [YminTrace]
    % Minimum Y coordinate to keep in the traffic trace (m)
    simParams = addNewParam(simParams,'YminTrace',-1,'Minimum Y coordinate to keep in the traffic trace (m)','double',fileCfg,varargin{1}{1});
    if simParams.YminTrace~=-1 && simParams.YminTrace<0
        error('Error: the value set for "appParams.YminTrace" is not valid');
    end
    
    % [YmaxTrace]
    % Maximum Y coordinate to keep in the traffic trace (m)
    simParams = addNewParam(simParams,'YmaxTrace',-1,'Maximum Y coordinate to keep in the traffic trace (m)','double',fileCfg,varargin{1}{1});
    if simParams.YmaxTrace~=-1 && simParams.YmaxTrace<0 && simParams.XmaxTrace<simParams.YminTrace
        error('Error: the value set for "appParams.YmaxTrace" is not valid');
    end
    
    % [positionTimeResolution]
    % Time resolution for the positioning update of the vehicles in the trace file (s)
    simParams = addNewParam(simParams,'positionTimeResolution',Tbeacon,'Time resolution for the positioning update of the vehicles in the trace file (s)','double',fileCfg,varargin{1}{1});
    if simParams.positionTimeResolution<=0
        error('Error: "appParams.positionTimeResolution" cannot be <= 0');
    end
    
    % Depending on the setting of "simParams.fileObstaclesMap", other parameters must
    % be set
    if simParams.fileObstaclesMap
        % [filenameObstaclesMap]
        % If the obstacles map file is used, this selects the file
        simParams = addNewParam(simParams,'filenameObstaclesMap','null.txt','File obstacles map name','string',fileCfg,varargin{1}{1});
        % Check that the file exists. If the file does not exist, the
        % simulation is aborted.
        fid = fopen(simParams.filenameObstaclesMap);
        if fid==-1
            fprintf('File obstacles map "%s" not found. Simulation Aborted.',simParams.filenameObstaclesMap);
        else
            fclose(fid);
        end
    end
else
    % [roadLength]
    % Length of the road to be simulated (m)
    simParams = addNewParam(simParams,'roadLength',4000,'Road Length (m)','double',fileCfg,varargin{1}{1});
    if simParams.roadLength<=0
        error('Error: "simParams.roadLength" cannot be <= 0');
    end
    
    % [roadWidth]
    % Width of each lane (m)
    simParams = addNewParam(simParams,'roadWidth',3.5,'Road Width (m)','double',fileCfg,varargin{1}{1});
    if simParams.roadWidth<0
        error('Error: "simParams.roadWidth" cannot be < 0');
    end
    
    % [NLanes]
    % Number of lanes per direction
    simParams = addNewParam(simParams,'NLanes',3,'Number of lanes per direction','integer',fileCfg,varargin{1}{1});
    if simParams.NLanes<=0
        error('Error: "simParams.NLanes" cannot be <= 0');
    end
    
    % [rho]
    % Density of vehicles (vehicles/km)
    simParams = addNewParam(simParams,'rho',100,'Density of vehicles (vehicles/km)','double',fileCfg,varargin{1}{1});
    if simParams.rho<=0
        error('Error: "simParams.rho" cannot be <= 0');
    end
    
    % [vMean]
    % Mean speed of vehicles (km/h)
    simParams = addNewParam(simParams,'vMean',114.23,'Mean speed of vehicles (Km/h)','double',fileCfg,varargin{1}{1});
    if simParams.vMean<0
        error('Error: "simParams.vMean" cannot be < 0');
    end
    
    % [vStDev]
    % Standard deviation of speed of vehicles (km/h)
    simParams = addNewParam(simParams,'vStDev',12.65,'Standard deviation of speed of vehicles (Km/h)','double',fileCfg,varargin{1}{1});
    if simParams.vStDev<0
        error('Error: "simParams.vStDev" cannot be < 0');
    end
end

% [Mborder]
% Margin for border effect removal (m)
simParams = addNewParam(simParams,'Mborder',0,'Margin for border effect removal (m)','integer',fileCfg,varargin{1}{1});
if simParams.Mborder < 0
    error('Error: "simParams.Mborder" cannot be negative.');
end

% [Technology]
% Choose if simulate LTE-V2V or 802.11p
% String: "LTEV2V" or "80211p"
simParams = addNewParam(simParams,'Technology','LTEV2V','Choose if simulate "LTEV2V" or "80211p"','string',fileCfg,varargin{1}{1});
% Check that the string is correct
if strcmpi(simParams.Technology,'LTEV2V')
    simParams.useLTE = true;
elseif strcmpi(simParams.Technology,'80211p')
    simParams.useLTE = false;
else
    error('"simParams.Technology" must be "LTEV2V" or "80211p"');
end


% [neighborsSelection]
% Choose whether to use significant neighbors selection
simParams = addNewParam(simParams,'neighborsSelection',false,'If using significant neighbors selection','bool',fileCfg,varargin{1}{1});
if simParams.neighborsSelection~=false && simParams.neighborsSelection~=true
    error('Error: "simParams.neighborsSelection" must be equal to false or true');
end

% [Mvicinity]
% Margin for trajectory vicinity (m)
simParams = addNewParam(simParams,'Mvicinity',10,'Margin for trajectory vicinity (m)','integer',fileCfg,varargin{1}{1});
if simParams.Mvicinity < 0
    error('Error: "simParams.Mvicinity" cannot be negative.');
end

fprintf('\n');

end
