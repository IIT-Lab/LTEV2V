function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentOrdered(Xvehicle,IDvehicle,BRid,Nbeacons,indexNoBorder)
% Benchmark Algorithm 102 (ORDERED ALLOCATION)

Nvehicles = length(IDvehicle(:,1));   % Number of vehicles

% Assign beacon resources ordered by X coordinates to vehicles
[~,indexOrder] = sort(Xvehicle);
IDvehicle = IDvehicle(indexOrder);
BRordered = mod((1:Nvehicles)'-1,Nbeacons)+1;

BRid(IDvehicle) = BRordered;

Nreassign = Nvehicles;
NreassignNoBorder = length(indexNoBorder);
Nunlocked = 0;
NunlockedNoBorder = 0;

end