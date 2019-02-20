function [BRid, oldBRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentAutoQC(IDvehicle,BRid,oldBRid,p,k,M,RXpower,Nbeacons,indexNoBorder,PnRB)
% Sensing-based autonomous resource reselection algorithm by Qualcomm Inc.
% Sensing period = beacon period
% L = 1 -> No averaging of energy across sensing periods
% p -> Probability of resource reselection
% k -> Number of best resources that are randomly selected
% M -> Hysteresys threshold (linear value)

% Reset number of successfully unlocked vehicles
Nunlocked = 0;
NunlockedNoBorder = 0;

% Reset number of successfully reassigned vehicles
Nreassign = 0;
NreassignNoBorder = 0;

% Find vehicles which will be reassigned (BRid = -3)
reassignIndex = find(BRid(IDvehicle)==-3);
reassignID = IDvehicle(reassignIndex);
reassignDim = length(reassignIndex);

% Build a sensing matrix where received power for each resource is stored
% rows -> index of reassignID
% columns -> BRid
sensingMatrix = zeros(reassignDim,Nbeacons);
for i = 1:reassignDim
    for j = 1:Nbeacons
        Indices = find(BRid(IDvehicle)==j);
        for z = 1:length(Indices)
            PRX = RXpower(reassignIndex(i),Indices(z));
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
rpMatrix = zeros(reassignDim,Nbeacons);

% Build matrix made of random permutations of the column indexes
sensingMatrixPerm = zeros(reassignDim,Nbeacons);
for i = 1:reassignDim
    rpMatrix(i,:) = randperm(Nbeacons);
    % Permute sensing matrix
    sensingMatrixPerm(i,:) = sensingMatrix(i,rpMatrix(i,:));
end

% Sort sensingMatrix in ascending order
[sensingMatrixOrdered, bestBRPerm] = sort(sensingMatrixPerm,2);

% Reorder bestBRid matrix
bestBR = zeros(reassignDim,Nbeacons);
for i = 1:reassignDim
    bestBR(i,:) = rpMatrix(i,bestBRPerm(i,:));
end
    
% If k exceeds the number of beacon resources, set k equal to Nbeacons
if k > Nbeacons
    k = Nbeacons;
end

% Keep the best k canditates
bestBR = bestBR(:,1:k);

for i = 1:reassignDim
    % Check if average power of currently selected resources is M dB 
    % more than the power measured on any of the k best candidates
    if oldBRid(reassignID(i))~=-1
        % If the vehicle was not blocked previously
        if sensingMatrix(i,oldBRid(reassignID(i))) > M*sensingMatrixOrdered(i,1)
            % If the condition is respected, randomly select a new resource  
            % among the best k candidates
            BRindex = randi(k);
            BR = bestBR(BRindex);
            BRid(reassignID(i))=BR;
            Nreassign = Nreassign + 1;
            if(isempty(find(indexNoBorder(:,1)==reassignIndex(i),1))==0)
                NreassignNoBorder = NreassignNoBorder + 1;
            end
        else
            % Else stick to the currently selected resources and use these
            % resources for future transmissions
            BRid(reassignID(i))=oldBRid(reassignID(i));
        end
    else
        % If the vehicle was blocked previously
        BRindex = randi(k);
        BR = bestBR(BRindex);
        BRid(reassignID(i))=BR;
        Nreassign = Nreassign + 1;
        if(isempty(find(indexNoBorder(:,1)==reassignIndex(i), 1))==0)
            NreassignNoBorder = NreassignNoBorder + 1;
        end
    end        
end

% Generate a vector to consider reselection
% 0 -> No reselection
% 1 -> Reselection (with probability p)
Nv = length(IDvehicle);
reselection = rand(Nv,1) > 1-p;

% Copy the old vector of BRs
oldBRid = BRid;

for i = 1:Nv
    if reselection(i)
        % If consider reselection, BRid is replaced with value -3 (sensing)
        BRid(IDvehicle(i))=-3;
    end
end

end

