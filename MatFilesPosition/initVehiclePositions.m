function [simParams,simValues] = initVehiclePositions(simParams,appParams)
% Function to initialize the positions of vehicles

if ~simParams.fileTrace
    
    % Scenario
    simValues.Xmin = 0;                           % Min X coordinate
    simValues.Xmax = simParams.roadLength;        % Max X coordinate
    simValues.Ymin = 0;                           % Min Y coordinate
                                                  % Max Y coordinate
    simValues.Ymax = 4*simParams.Mborder+2*simParams.NLanes*simParams.roadWidth;                  
    
    vMeanMs = simParams.vMean/3.6;                % Mean vehicle speed (m/s)
    vStDevMs = simParams.vStDev/3.6;              % Speed standard deviation (m/s)
    simParams.rhoM = simParams.rho/1e3;           % Average vehicle density (vehicles/m)
    
    Nvehicles = round(simParams.rhoM*simValues.Xmax);   % Number of vehicles
    simValues.IDvehicle(:,1) = 1:Nvehicles;             % Vector of IDs
    simValues.maxID = Nvehicles;                        % Maximum vehicle's ID
    
    % Generate X coordinates of vehicles (uniform distribution)
    simValues.XvehicleReal = simValues.Xmax.*rand(Nvehicles,1);
    
    % Generate driving direction
    % 0 -> from left to right
    % 1 -> from right to left
    simValues.direction = rand(Nvehicles,1) > 0.5;
    right = find(simValues.direction==0);
    left = find(simValues.direction);
     
    % Generate Y coordinates of vehicles (distributed among Nlanes)
    simValues.YvehicleReal = zeros(Nvehicles,1);
    for i = 1:length(right)
        lane = randi(simParams.NLanes);
        simValues.YvehicleReal(right(i)) = 2*simParams.Mborder+lane*simParams.roadWidth;
    end
    for i = 1:length(left)
        lane = randi([simParams.NLanes+1 2*simParams.NLanes]);
        simValues.YvehicleReal(left(i)) = 2*simParams.Mborder+lane*simParams.roadWidth;
    end
    
    % Assign speed to vehicles
    simValues.v = abs(vMeanMs + vStDevMs.*randn(Nvehicles,1));
    
    % Time resolution of position update corresponds to the beacon period
    simParams.positionTimeResolution = appParams.Tbeacon;
    simValues.timeResolution = simParams.positionTimeResolution;
    
else
    
    % Call function to load the traffic trace up to the selected simulation
    % time and, if selected, take only a portion of the scenario
    [dataLoaded,simParams] = loadTrafficTrace(simParams);
    
    % Call function to interpolate the traffic trace (if needed)
    [simValues,simParams] = interpolateTrace(dataLoaded,simParams,appParams.Tbeacon);
    
    % Round time column (representation format)
    simValues.dataTrace(:,1) = round(simValues.dataTrace(:,1)*100)/100;

    % Find trace details (Xmin,Xmax,Ymin,Ymax,maxID)
    simValues.Xmin = min(simValues.dataTrace(:,3));     % Min X coordinate Trace
    simValues.Xmax = max(simValues.dataTrace(:,3));     % Max X coordinate Trace
    simValues.Ymin = min(simValues.dataTrace(:,4));     % Min Y coordinate Trace
    simValues.Ymax = max(simValues.dataTrace(:,4));     % Max Y coordinate Trace
    simValues.maxID = max(simValues.dataTrace(:,2));    % Maximum vehicle's ID
    
    % Call function to read vehicle positions from file at time zero
    [simValues.XvehicleReal, simValues.YvehicleReal, simValues.IDvehicle] = updatePositionFile(0,simValues.dataTrace,0);
    
end
