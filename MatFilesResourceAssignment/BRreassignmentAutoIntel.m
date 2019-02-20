function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentAutoIntel(IDvehicle,BRid,scheduledIndex,MBest,RXpower,Nbeacons,indexNoBorder,PnRB)
% Sensing-based autonomous resource reselection algorithm by Intel Corp.
% Resources are allocated for a Resource Reselection Period (SPS)
% Sensing is performed in the last beacon period of the SPS period
% Congestion map: M best transmission hypothesis
% Random selection of one of the M best candidates

% Reset number of successfully unlocked vehicles
Nunlocked = 0;
NunlockedNoBorder = 0;

% Reset number of successfully reassigned vehicles
Nreassign = 0;
NreassignNoBorder = 0;

% Number of vehicles which perform reselection
Nscheduled = length(scheduledIndex);
scheduledID = IDvehicle(scheduledIndex);

% Build a sensing matrix where received power for each resource is stored
% rows -> index of vehicles which perform sensing
% columns -> BRid
sensingMatrix = zeros(Nscheduled,Nbeacons);
for i = 1:Nscheduled
    for j = 1:Nbeacons
        Indices = find(BRid(IDvehicle)==j);
        for z = 1:length(Indices)
            PRX = RXpower(scheduledIndex(i),Indices(z));
            sensingMatrix(i,j) = sensingMatrix(i,j) + PRX;
        end
        if sensingMatrix(i,j) < PnRB/10
            % If the received power measured on that resource is lower than
            % the receiver sensibility
            sensingMatrix(i,j) = 0;
        end
        % Add noise contribution to measured power
        sensingMatrix(i,j) = sensingMatrix(i,j) + PnRB;
    end
end

% Create random permutation of the column indexes of sensingMatrix in
% order to avoid the ascending order on the indexes of cells with the
% same value (sort effect) -> minimize the probability of choosing the same
% resource
rpMatrix = zeros(Nscheduled,Nbeacons);

% Build matrix made of random permutations of the column indexes
sensingMatrixPerm = zeros(Nscheduled,Nbeacons);
for i = 1:Nscheduled
    rpMatrix(i,:) = randperm(Nbeacons);
    % Permute sensing matrix
    sensingMatrixPerm(i,:) = sensingMatrix(i,rpMatrix(i,:));
end

% Sort sensingMatrix in ascending order
[~, bestBRPerm] = sort(sensingMatrixPerm,2);

% Reorder bestBRid matrix
bestBR = zeros(Nscheduled,Nbeacons);
for i = 1:Nscheduled
    bestBR(i,:) = rpMatrix(i,bestBRPerm(i,:));
end

% If MBest exceeds the number of beacon resources, set it equal to
% Nbeacons - 1 (exclude the resource used previously)
if MBest >= Nbeacons
    MBest = Nbeacons - 1;
end

% Keep the best M canditates
bestBR = bestBR(:,1:MBest);

for i = 1:Nscheduled
    BRindex = randi(MBest);
    BR = bestBR(BRindex);
    BRid(scheduledID(i))=BR;
    Nreassign = Nreassign + 1;
    if(isempty(find(indexNoBorder(:,1)==scheduledIndex(i), 1))==0)
        NreassignNoBorder = NreassignNoBorder + 1;
    end
end

end

