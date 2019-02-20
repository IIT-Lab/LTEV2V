function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentRandom(IDvehicle,BRid,Nbeacons,indexNoBorder)
% Benchmark Algorithm 101 (RANDOM ALLOCATION)

Nvehicles = length(IDvehicle(:,1));   % Number of vehicles      

% Assign a random beacon resource to vehicles
BRid(IDvehicle) = randi(Nbeacons,Nvehicles,1);
Nreassign = Nvehicles;
NreassignNoBorder = length(indexNoBorder);
Nunlocked = 0;
NunlockedNoBorder = 0;

end