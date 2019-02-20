function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentControlledMaxReuse(IDvehicle,BRid,scheduledID,allNeighborsID,NbeaconsT,NbeaconsF,indexNoBorder)
% [CONTROLLED CASE WITH MAXIMUM REUSE]
% Forcedly reassign BRs to the group of vehicles that have been scheduled,
% using the maximum possible distance for reuse

% Find IDs of new vehicles in the scenario (initially blocked)
newVehicleID = find(BRid==-1);

% Add new vehicles to the scheduled IDs (without repetitions)
scheduledID = unique(vertcat(scheduledID,newVehicleID));

% Number of scheduled vehicles
Nscheduled = length(scheduledID);
p = randperm(Nscheduled);

% Update vector of resources
BRid(scheduledID)=-1;

% Update matrix of allNeighborsBRid
allNeighborsBRid = BRidmatrix(allNeighborsID,BRid);

% Find BRT and BRF matrices corresponding to allNeighborsBRid
indexNOT = allNeighborsBRid <=0;
allNeighborsBRTid = mod(allNeighborsBRid-1,NbeaconsT)+1;
allNeighborsBRFid = ceil(allNeighborsBRid/NbeaconsT);
allNeighborsBRTid(indexNOT) = -1;
allNeighborsBRFid(indexNOT) = -1;

% Assign BRs to scheduled vehicles
for i = 1:Nscheduled
    z = p(i);
    ID = scheduledID(z,1);
    % Find the correspondent index in IDvehicle
    indexVehicle = IDvehicle==ID;
    % Initialize array of occupied BRTs
    beaconArrayT = zeros(NbeaconsT,1);
    foundT = 0;
    % Check BRids of all vehicles in ordered distance
    for j=1:length(IDvehicle)-1
        BRT = allNeighborsBRTid(indexVehicle,j);
        if BRT > 0
            % BRT is occupied
            beaconArrayT(BRT) = 1;
            if nnz(beaconArrayT) == NbeaconsT
                % If all BRTs are occupied, choose the last one (maximum
                % distance) and check the occupation of BRFs
                foundT = 1;
                neighborsIndex = find(allNeighborsBRTid(indexVehicle,:)==BRT);
                % Initialize array of occupied BRFs
                beaconArrayF = zeros(NbeaconsF,1);
                foundF = 0;
                % Start searching for the best BRF (the one used at higher distance)
                for k = 1:length(neighborsIndex)
                    BRF = allNeighborsBRFid(indexVehicle,neighborsIndex(k));
                    % BRF is occupied
                    beaconArrayF(BRF) = 1;
                    if nnz(beaconArrayF) == NbeaconsF
                        % All BRFs are used, choose the last one and update BRid
                        BRid(ID) = (BRF-1)*NbeaconsT+BRT;
                        foundF = 1;
                        break
                    end     
                end
                break
            end
        end
    end
    
    % If there are more than one free BRTs
    if ~foundT
        % Find free BRTs
        freeBRTs = find(beaconArrayT==0);
        % Choose a random BRT among the free ones
        index = randi(length(freeBRTs));
        BRT = freeBRTs(index);
        % Choose a random BRF among all NbeaconsF
        BRF = randi(NbeaconsF);
    % If there are more than one free BRFs
    elseif foundT && ~foundF
        % Find free BRFs
        freeBRFs = find(beaconArrayF==0);
        % Choose a random BRF among the free ones
        index = randi(length(freeBRFs));
        BRF = freeBRFs(index);
    end
    
    % Update BRid of vehicle ID
    BRid(ID) = (BRF-1)*NbeaconsT+BRT;
    
    % Update allNeighborsBRid, allNeighborsBRTid, allNeighborsBRFid
    indices = allNeighborsID==ID;
    allNeighborsBRid(indices) = BRid(ID);
    allNeighborsBRTid(indices) = BRT;
    allNeighborsBRFid(indices) = BRF;
end

Nreassign = Nscheduled;
NreassignNoBorder = length(intersect(IDvehicle(indexNoBorder),scheduledID));
Nunlocked = 0;
NunlockedNoBorder = 0;

end