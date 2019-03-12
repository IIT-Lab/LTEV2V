function [XvehicleReal,YvehicleReal,IDvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePositionFile(time,dataTrace,oldIDvehicle)
% Update position of vehicles from file

fileIndex = find(dataTrace(:,1)==time);
IDvehicle = dataTrace(fileIndex,2);
XvehicleReal = dataTrace(fileIndex,3);
YvehicleReal = dataTrace(fileIndex,4);

% Sort IDvehicle, XvehicleReal and YvehicleReal by IDvehicle
[IDvehicle,indexOrder] = sort(IDvehicle);
XvehicleReal = XvehicleReal(indexOrder);
YvehicleReal = YvehicleReal(indexOrder);

[~,indexNewVehicles] = setdiff(IDvehicle,oldIDvehicle,'stable');

% Find IDs of vehicles that are exiting the scenario
IDvehicleExit = setdiff(oldIDvehicle,IDvehicle);

% Find indices of  vehicles in IDvehicle that are both in IDvehicle and OldIDvehicle
indexOldVehicles = find(ismember(IDvehicle,oldIDvehicle));

% Find indices of vehicles in OldIDvehicle that are both in IDvehicle and OldIDvehicle
indexOldVehiclesToOld = find(ismember(oldIDvehicle,IDvehicle));

end