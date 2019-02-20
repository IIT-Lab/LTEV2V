function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentControlled(IDvehicle,previouslyReassigned,errorMatrix,distance,BRid,Nbeacons,indexNoBorder,Rreuse,randomOrder)
% [CONTROLLED CASE]
% Reassign BRs to blocked vehicles and vehicles that have been advised to
% reassign BR due to errors caused by high interference

Nerrors = length(errorMatrix(:,1));                      % Number of errors

% Initialize IDs of vehicles that have already been reassigned
% (useful for BR algorithm with scheduled (2) 
Nprevious = length(find(previouslyReassigned(:,1)));
if Nprevious~=0
    alreadyReassigned = zeros(Nprevious+Nerrors,1);
    alreadyReassigned(1:Nprevious,1) = previouslyReassigned;
    w = Nprevious+1;
else
    alreadyReassigned = zeros(Nerrors,1);
    w = 1;
end
    
Nunlocked = 0;                          % Reset number of unlocked vehicles
NunlockedNoBorder = 0;

Nreassign = 0;                   % Reset number of successful reassignments
NreassignNoBorder = 0;

% Reassign BRs to blocked vehicles
blockedIndex = find(BRid(IDvehicle)==-1);           % Find blocked vehicles
Nb = length(blockedIndex);

if Nb~=0
    p1 = randperm(Nb);                 % Generate random permutation vector
    % Assign BRs to blocked vehicles
    for i = 1:Nb
        z = p1(i);
        ID = IDvehicle(blockedIndex(z,1));
        % Check if the vehicle has already been reassigned
        index = find(alreadyReassigned(:,1)==ID,1);
        if isempty(index)==1
            % Add ID to already reassigned vehicles
            alreadyReassigned(w,1) = ID;
            w = w+1;
            p2 = randperm(Nbeacons);       % Generate random permutation vector
            for j = 1:Nbeacons  % Check all possible BRs
                if ~randomOrder
                    BR = j;
                else
                    BR = p2(j);
                end
                found = 0;  % Becomes 1 if there is a vehicle using that BR within reuse distance
                intIndex = find(BRid(IDvehicle)==BR);
                for k = 1:length(intIndex)
                    if distance(blockedIndex(z,1),intIndex(k))<Rreuse
                        found = 1;
                        break
                    end
                end
                if found
                    BRid(ID) = -1;  % BR not available (blocked)
                else
                    BRid(ID) = BR;  % Assign BR
                    Nunlocked = Nunlocked + 1;
                    if(isempty(find(indexNoBorder(:,1)==blockedIndex(z,1),1))==0)
                        NunlockedNoBorder = NunlockedNoBorder + 1;
                    end
                    break
                end
            end
        end
    end
end

% Reassign BRs to vehicles that transmit wrongly decoded beacons
if Nerrors~=0
    p3 = randperm(Nerrors);            % Generate random permutation vector
    % Assign BRs to vehicle that transmit packet with errors
    for i = 1:Nerrors
        z = p3(i);
        ID = errorMatrix(z,2);
        % Find the correspondent index in IDvehicle
        IndexVehicle = find(IDvehicle==ID);   
        % Check if the vehicle has already been reassigned
        index = find(alreadyReassigned(:,1)==ID,1);
        if isempty(index)==1
            % Add ID to already reassigned vehicles
            alreadyReassigned(w,1) = ID;
            w = w+1;
            p4 = randperm(Nbeacons);   % Generate random permutation vector
            % Try to reassign BR to ID
            for j = 1:Nbeacons   % Check all possible BRs
                if ~randomOrder
                    BR = j;
                else
                    BR = p4(j);
                end
                found = 0;  % Becomes 1 if there is a vehicle using that BR within reuse distance
                intIndex = find(BRid(IDvehicle)==BR);
                for k = 1:length(intIndex)
                    if distance(IndexVehicle,intIndex(k))<Rreuse
                        found = 1;
                        break
                    end
                end
                if found
                    BRid(ID) = -1;  % BR not available (blocked)
                else
                    BRid(ID) = BR;  % Assign BR
                    Nreassign = Nreassign + 1;
                    if(isempty(find(indexNoBorder(:,1)==IndexVehicle,1))==0)
                        NreassignNoBorder = NreassignNoBorder + 1;
                    end
                    break
                end
            end
        end
    end
end

end