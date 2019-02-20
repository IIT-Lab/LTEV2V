function [BRid, Nassign, NassignNoBorder] = firstAssignment(IDvehicle,BRid,distance,Nbeacons,indexNoBorder,Rreuse,randomOrder)
% First BRs assignment algorithm

Nvehicles = length(IDvehicle(:,1));   % Number of vehicles
p1 = randperm(Nvehicles);             % Generate random permutation vector
Nassign = 0;                          % Reset number of successful assignments
NassignNoBorder = 0;

% Assign BRs to vehicles
for i = 1:Nvehicles
    z = p1(i);
    ID = IDvehicle(z,1);
    p2 = randperm(Nbeacons);       % Generate random permutation vector
    for j = 1:Nbeacons             % Check all possible BRs
        if ~randomOrder
            BR = j;
        else
            BR = p2(j);
        end
        found = 0;  % Becomes 1 if there is a vehicle using that BR within reuse distance
        IDinterferers = find(BRid==BR);
        for k=1:length(IDinterferers)
            index = IDvehicle==IDinterferers(k);
            if distance(z,index)<Rreuse
                found = 1;
                break
            end
        end
        if found
            BRid(ID) = -1;  % BR not available (blocked)
        else
            BRid(ID) = BR;  % Assign BR
            Nassign = Nassign + 1;
            if(isempty(find(indexNoBorder(:,1)==z, 1))==0)
                NassignNoBorder = NassignNoBorder + 1;
            end
            break
        end
    end
end

end