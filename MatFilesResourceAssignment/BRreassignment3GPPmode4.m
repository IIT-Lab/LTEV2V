function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder, sensingMatrix, knownUsedMatrix, resReselectionCounter] = BRreassignment3GPPmode4(IDvehicle,BRid,sensingMatrix,knownUsedMatrix,resReselectionCounter,neighborsID,errorSCImatrix,simParams,timeNextPacket,RXpower,IBEmatrix,Nbeacons,NbeaconsF,NbeaconsT,indexNoBorder,Ksi,PtxERP_RB,PnRB,appParams)
% Sensing-based autonomous resource reselection algorithm (3GPP MODE 4)
% as from 3GPP TS 36.321 and TS 36.213
% Resources are allocated for a Resource Reselection Period (SPS)
% Sensing is performed in the last 1 second
% Map of the received power and selection of the best 20% transmission hypothesis
% Random selection of one of the M best candidates
% The selection is rescheduled after a random period, with random
% probability controlled by the input parameter 'probResKeep'

% The best 20% (modifiable by input) is selected as pool as in TS 36.213
% If T1==1 and T2==100, Mbest is the 20% of all beacon resources
% In the case T1>1 and/or T2<100, Mbest is the 20% of the consequent number
% of resources
MBest = ceil(Nbeacons * ((simParams.subframeT2Mode4-simParams.subframeT1Mode4+1)/100) * simParams.ratioSelectedMode4);

% Set nVehicles
nVehicles = length(IDvehicle);

% Reset number of successfully unlocked vehicles
Nunlocked = 0;
NunlockedNoBorder = 0;

% Reset number of successfully reassigned vehicles
Nreassign = 0;
NreassignNoBorder = 0;

%% Update the sensing matrix
% The sensingMatrix is a 3D matrix with
% 1st D -> Number of values to be stored in the time domain, corresponding
%          to the standard duration of 1 second, of size ceil(1/Tbeacon)
% 2nd D -> BRid, of size Nbeacons
% 3rd D -> IDs of vehicles
% The matrix is first shifted in the third dimension (time)
% and zeros are introduced in the first position (of the 1st dimension)
% and then the new values of the received power for each resource are stored

% Shift the sensing matrix and reset the the values in the first position
% of the 1st dimension
sensingMatrix = circshift(sensingMatrix,1,1);
sensingMatrix(1,:,:) = zeros(length(sensingMatrix(1,:,1)),length(sensingMatrix(1,1,:)));

% Calculate the beacon resources used in the time and frequency domains
% Find not assigned BRid
idNOT = BRid<=0;
% Calculate BRidT = vector of BRid in the time domain
BRidT = mod(BRid-1,NbeaconsT)+1;
BRidT(idNOT) = -1;
% Calculate BRidF = vector of BRid in the frequency domain
BRidF = ceil(BRid/NbeaconsT);
BRidF(idNOT) = -1;

% Cycle that calculates per each vehicle the sensed power
for iV = 1:nVehicles
    % The subframe of beacon generation is identified, modulo NbeaconsT
    indexBRTbeaconGen = mod(ceil(timeNextPacket(IDvehicle(iV))/0.001),NbeaconsT);
    % Then the window T1-T2 is identified within 1-MbeaconsT
    T1 = mod(indexBRTbeaconGen + simParams.subframeT1Mode4,NbeaconsT);
    T2 = mod(indexBRTbeaconGen + simParams.subframeT2Mode4,NbeaconsT);
    % Cycle that calculates (per each vehicle) per each resource the sensed power
    for iBR = 1:Nbeacons
        % Evaluate the time and frequency indexes corresponding to the resource
        BRTi = mod(iBR-1,NbeaconsT)+1;
        BRFi = ceil(iBR/NbeaconsT);
        % In the case T1>1 and/or T2<100, if the subframe cannot be used
        % the total power is set to infinite
        %%%%timeNextPacket
        if simParams.subframeT1Mode4>1 || simParams.subframeT2Mode4<100
            if (T2>T1 && (BRTi<T1 || BRTi>T2)) || ...
               (T2<T1 && (BRTi>T1 && BRTi<T2))
                sensingMatrix(1,iBR,IDvehicle(iV)) = inf;
                continue;
            end
        end
        % Init the vector of received power in the same subframe of this resource
        rxPsums = zeros(NbeaconsF,1);
        % Cycle over the vehicles transmitting in the same subframe
        intIndex = find(BRidT(IDvehicle)==BRTi);
        for k = 1:length(intIndex)
            % Find which BRF is used by the interferer
            BRFother = BRidF(IDvehicle(intIndex(k)));
            % Separate all other vehicles to itself
            if IDvehicle(intIndex(k))~=IDvehicle(iV)
                % If not itself, add the received power
                rxPsums(BRFother) = rxPsums(BRFother) + RXpower(iV,intIndex(k));
            else 
                % Including itself allows simulating full duplex devices
                % If itself, add the Tx power multiplied by Ksi (set to inf 
                %       if the devices are half duplex)
                rxPsums(BRFother) = rxPsums(BRFother) + Ksi*PtxERP_RB(IDvehicle(iV));
            end
        end
        % Find total received power using IBE
        sensingMatrix(1,iBR,IDvehicle(iV)) = IBEmatrix(BRFi,:)*rxPsums;
        if sensingMatrix(1,iBR,IDvehicle(iV)) < PnRB
            % If the received power measured on that resource is lower than
            % a threshold, it is assumed that no power is measured
            sensingMatrix(1,iBR,IDvehicle(iV)) = 0;
        end
    end
end

% Cycle that updates per each vehicle and BR the knownUsedMatrix
%  knownUsedMatrix = zeros(appParams.Nbeacons,simValues.maxID);
for indexVrx = 1:nVehicles
    idVrx = IDvehicle(indexVrx);
    for indexNeighbors = 1:length(neighborsID(indexVrx,:))
       idVtx = neighborsID(indexVrx,indexNeighbors);
       if idVtx==0
           break;
       end
       BRtx = BRid(idVtx);
       if errorSCImatrix(indexVrx,indexNeighbors) == 0 && BRtx >= 1 && ...
              knownUsedMatrix(BRtx,idVrx) < resReselectionCounter(idVtx)
        knownUsedMatrix(BRtx,idVrx) = resReselectionCounter(idVtx);
       end
    end
end

% -- A = squeeze(sensingMatrix(1,:,1));
% A = squeeze(sum(sensingMatrix(:,:,1))/10);
% B = reshape(A,NbeaconsT,Nbeacons/NbeaconsT);
% B(B(:,:)==0)=PnRB;
% -- bar(10*log10(B)+30);
% C = squeeze(knownUsedMatrix(:,1));
% D = reshape(C,NbeaconsT,Nbeacons/NbeaconsT);
% D = -20+10*(D>0)
% bar([10*log10(B)+30 D]);

%% Update the resReselectionCounter and evaluate which vehicles need reselection
% Calculate scheduledID
scheduledID = find(resReselectionCounter==0);

% Define the number of vehicles which perform reselection
Nscheduled = length(scheduledID);

% Update resReselectionCounter
% Reduce the counter by one
% Those that reached 0 need to be reset between min and max
resReselectionCounter(IDvehicle) = resReselectionCounter(IDvehicle)-1;
resReselectionCounter(scheduledID) = (simParams.minRandValueMode4-1) + randi((simParams.maxRandValueMode4-simParams.minRandValueMode4)+1,1,Nscheduled);

% For those that have the counter reaching 0, a random variable should be drawn
% to define if the resource is kept or not, based on the input parameter probResKeep
keepRand = rand(1,Nscheduled);
if simParams.probResKeep>0
    scheduledID = scheduledID(keepRand >= simParams.probResKeep);
end % else all vehicles with the counter reaching zero perform the reselection

% Update the number of vehicles which perform reselection
Nscheduled = length(scheduledID);

%% Perform the reselection
for iV = 1:Nscheduled

    % Select the sensing matrix only for the vehicles that perform reallocation
    sensingMatrixScheduled = sum(sensingMatrix(:,:,scheduledID(iV)),1)/length(sensingMatrix(:,1,1));
    knownUsedMatrixScheduled = knownUsedMatrix(:,scheduledID(iV))';

    % Create random permutation of the column indexes of sensingMatrix in
    % order to avoid the ascending order on the indexes of cells with the
    % same value (sort effect) -> minimize the probability of choosing the same
    % resource
    rpMatrix = randperm(Nbeacons);

    % Build matrix made of random permutations of the column indexes
    % Permute sensing matrix
    sensingMatrixPerm = sensingMatrixScheduled(rpMatrix);
    knownUsedMatrixPerm = knownUsedMatrixScheduled(rpMatrix);

    % Now perform sorting and relocation taking into account the threshold on RSRP
    % Please note that the sensed power is on a per beacon resource basis,
    % whereas simParams.powerThresholdMode4 is on a resource element basis,
    % thus a conversion is required (recall: 12 subcarriers per RB)
    powerThreshold = simParams.powerThresholdMode4 * (12*appParams.RBsBeacon);
    % The cycle is stopped internally; a max of 100 is used to avoid
    % infinite loops in case of bugs
    while powerThreshold < 100
        % If the number of acceptable BRs is lower than MBest,
        % powerThreshold is increased by 3 dB
        usableBRs = (sensingMatrixPerm<powerThreshold) | (knownUsedMatrixPerm<1);
        if sum( usableBRs ) < MBest
            powerThreshold = powerThreshold * 2;
        else
            break;
        end
    end        
    
    % To mark unacceptable RB as occupied, their power is increased by a large value, 
    % which we selected as the maxmum of the PtxERP_RB vector
%    sensingMatrixPermTemp = sensingMatrixPerm;
    sensingMatrixPerm = sensingMatrixPerm + (1-usableBRs) * max(PtxERP_RB);
    %bar([10*log10(sort(sensingMatrixPerm)') 10*log10(sort((sensingMatrixPerm + (1-usableBRs) * PtxERP_RB))')])
    
%     firstDifferent = find(sort(sensingMatrixPermTemp)-sort(sensingMatrixPerm)~=0,1);
%     if firstDifferent < MBest
%         A = squeeze(sum(sensingMatrix(:,:,scheduledID(iV)))/10);
%         B = reshape(A,NbeaconsT,Nbeacons/NbeaconsT);
%         B(B(:,:)==0)=PnRB;
%         %-- bar(10*log10(B)+30);
%         C = squeeze(knownUsedMatrix(:,scheduledID(iV)));
%         D = reshape(C,NbeaconsT,Nbeacons/NbeaconsT);
%         D = -20+10*(D>0);
%         bar([10*log10(B)+30 D]); 
%         hold on
%         E = sort(sensingMatrixPerm);
%         powref = E(MBest);
%         plot([-1 101],[10*log10(powref)+30 10*log10(powref)+30],'--b')
%         plot([-1 101],[10*log10(powerThreshold)+30 10*log10(powerThreshold)+30],'-')
%         D = D;
%     end
    
    % Sort sensingMatrix in ascending order
    [~, bestBRPerm] = sort(sensingMatrixPerm);

    % Reorder bestBRid matrix
    bestBR = rpMatrix(bestBRPerm);

    % Keep the best M canditates
    bestBR = bestBR(1:MBest);

    % Reassign
    BRindex = randi(MBest);
    BR = bestBR(BRindex);
    BRid(scheduledID(iV))=BR;
    Nreassign = Nreassign + 1;
    scheduledIndex = find(IDvehicle==scheduledID(iV));
    if(isempty(find(indexNoBorder(:,1)==scheduledIndex, 1))==0)
        NreassignNoBorder = NreassignNoBorder + 1;
    end
end

% Reduce the knownUsedMatrix by 1 (not a probloem if it goes below 0
knownUsedMatrix = knownUsedMatrix -1;

end

