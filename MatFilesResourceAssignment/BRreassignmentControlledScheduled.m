function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentControlledScheduled(IDvehicle,BRid,scheduledID,distance,errorMatrix,Nbeacons,indexNoBorder,Rreuse,randomOrder)
% [CONTROLLED CASE WITH SCHEDULED REASSIGNMENT]
% Forcedly reassign BRs to the group of vehicles that have been scheduled,
% then reassign BRs to blocked vehicles and to vehicles that have been
% advised to reassign BR due to errors caused by high interference

% Update vector of resources
BRid(scheduledID)=-1;                 % Free BRs used by scheduled vehicles
Nscheduled = length(scheduledID);     % Number of scheduled vehicles
p1 = randperm(Nscheduled);            % Generate random permutation vector
NreassignScheduled = 0;               % Reset number of successful reassignments
NreassignScheduledNoBorder = 0;

if Nscheduled~=0
    % Assign BRs to scheduled vehicles
    for i = 1:Nscheduled
        z = p1(i);
        ID = scheduledID(z,1);
        % Find the correspondent index in IDvehicle
        IndexVehicle = find(IDvehicle==ID);
        p2 = randperm(Nbeacons);       % Generate random permutation vector
        for j = 1:Nbeacons             % Check all possible BRs
            if ~randomOrder
                BR = j;
            else
                BR = p2(j);
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
                NreassignScheduled = NreassignScheduled + 1;
                if(isempty(find(indexNoBorder(:,1)==IndexVehicle,1))==0)
                    NreassignScheduledNoBorder = NreassignScheduledNoBorder + 1;
                end
                break
            end
        end
    end
end

% Initialize IDs of vehicles that have already been reassigned
previouslyReassigned = scheduledID;

% Call function to reassign resources to blocking and error events in
% network-controlled LTE-V2V
[BRid, NreassignControlled, NreassignControlledNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentControlled(IDvehicle,previouslyReassigned,errorMatrix,distance,BRid,Nbeacons,indexNoBorder,Rreuse,randomOrder);

% Sum the number of scheduled reassignments + controlled reassignments
Nreassign = NreassignScheduled + NreassignControlled;
NreassignNoBorder = NreassignScheduledNoBorder + NreassignControlledNoBorder;

end