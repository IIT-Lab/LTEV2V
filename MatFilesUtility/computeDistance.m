function [distance,awarenessID,neighborsDistance,neighborsID,allNeighborsID] = computeDistance(Xvehicle,Yvehicle,IDvehicle,Raw,RawMax)
% Compute distances and create awareness matrix

% Compute distance matrix
distance = sqrt((Xvehicle - Xvehicle').^2+(Yvehicle - Yvehicle').^2);

% Sort distance matrix and indices in ascending order (by columns)
[neighborsDistance, neighborsIndex] = sort(distance,2);
neighborsDistance(:,1) = [];
neighborsIndex(:,1) = [];

% Vehicles in order of distance
allNeighborsID = IDvehicle(neighborsIndex);

% Vehicles in the maximum awareness range
neighborsID = (neighborsDistance < RawMax).*IDvehicle(neighborsIndex);

% Vehicles in awareness range
awarenessID = (neighborsDistance < Raw).*neighborsID;

% Keep only the distance of neighbors up to the maximum awareness range
neighborsDistance = (neighborsDistance < RawMax).*neighborsDistance;

end