function [Xvehicle,Yvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePosition(Xvehicle,Yvehicle,IDvehicle,v,direction,T,Xmax)
% Update vehicles position (when not using File Trace)
% (if a vehicle moves outside the scenario, enters by the other side)

Xvehicle = (~direction).*mod(Xvehicle + v*T,Xmax) + direction.*mod(Xvehicle - v*T,Xmax);

% Return indices
indexNewVehicles = [];
indexOldVehicles = IDvehicle;
indexOldVehiclesToOld = IDvehicle;
IDvehicleExit = [];

end