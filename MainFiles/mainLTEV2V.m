function [simValues,outputValues,appParams,simParams,phyParams,outParams] = mainLTEV2V(appParams,simParams,phyParams,outParams,simValues,outputValues)
% The function mainLTEV2V() performs the LTE-V2V simulation

%% First Radio Resources Assignment

% Initialize Tx power per RB vector
phyParams.maxPtxERP_RB = phyParams.PtxERP_RB; % Max PtxERP
phyParams.PtxERP_RB = phyParams.PtxERP_RB*ones(simValues.maxID,1);

% Number of groups for position update
NPosUpdate = round(simParams.Tupdate/appParams.Tbeacon);

% Assign update period to all vehicles (introduce a position update delay)
posUpdateAllVehicles = randi(NPosUpdate,simValues.maxID,1);

% Copy real coordinates of vehicles
simValues.Xvehicle = simValues.XvehicleReal;
simValues.Yvehicle = simValues.YvehicleReal;

% Find vehicles not at the borders
indexNoBorder = find((simValues.XvehicleReal(:,1)>=(simValues.Xmin+simParams.Mborder) & simValues.XvehicleReal(:,1)<=(simValues.Xmax-simParams.Mborder)) & (simValues.YvehicleReal(:,1)>=(simValues.Ymin+simParams.Mborder) & simValues.YvehicleReal(:,1)<=(simValues.Ymax-simParams.Mborder)));

% Call function to compute distances
% Distance matrix has dimensions equal to IDvehicle x IDvehicle in order to
% speed up the computation (only vehicles present at the considered instant)
% distance(i,j): distance from vehicle with index i to vehicle with index j
[distance,~,~,~,allNeighborsID] = computeDistance(simValues.Xvehicle,simValues.Yvehicle,simValues.IDvehicle,phyParams.Raw,phyParams.RawMax);

% Save distance matrix
distanceRealOld = distance;

% Initialize array of old of coordinates
XvehicleRealOld = simValues.XvehicleReal;
YvehicleRealOld =  simValues.YvehicleReal;

% Initialize array of old angles
angleOld = zeros(length(XvehicleRealOld),1);

% Initialize BRid
BRid = -2*ones(simValues.maxID,1);

if simParams.BRAlgorithm==101
    % Call Benchmark Algorithm 101 (RANDOM ALLOCATION)
    [BRid] = BRreassignmentRandom(simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
    
elseif simParams.BRAlgorithm==102
    % Call Benchmark Algorithm 102 (ORDERED ALLOCATION)
    [BRid] = BRreassignmentOrdered(simValues.XvehicleReal,simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
    
elseif simParams.BRAlgorithm==7
    % Call first assignment of BR Algorithm 7 (CONTROLLED with MAXIMUM REUSE)
    scheduledID = simValues.IDvehicle;
    [BRid] = BRreassignmentControlledMaxReuse(simValues.IDvehicle,BRid,scheduledID,allNeighborsID,appParams.NbeaconsT,appParams.NbeaconsF,indexNoBorder,1);
else
    % First BRs assignment (Always CONTROLLED)
    % BRid -> BR assigned
    %   -1 -> blocked
    %   -2 -> vehicle outside the scenario (unknown position)
    [BRid] = firstAssignment(simValues.IDvehicle,BRid,distance,appParams.Nbeacons,indexNoBorder,phyParams.Rreuse,simParams.randomOrder);
end

if simParams.BRAlgorithm==5
    % Initialization of oldBRid for the first cycle
    oldBRid = BRid;
end

if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==6 || simParams.BRAlgorithm==7
    % Number of groups for scheduled resource reassignment (BRAlgorithm=2 or 6)
    NScheduledReassign = round(simParams.Treassign/appParams.Tbeacon);
    
    % Assign update period to vehicles (BRAlgorithm=2 or 6)
    scheduledReassign = randi(NScheduledReassign,simValues.maxID,1);
end

if simParams.BRAlgorithm==8
    % Find min and max values for random counter (BRAlgorithm=8)
    [simParams.minRandValueMode4,simParams.maxRandValueMode4] = findRandValueMode4(appParams.Tbeacon,simParams);
    
    % Initialize reselection counter (BRAlgorithm=8)
    resReselectionCounter = Inf*ones(simValues.maxID,1);
    % Generate values for vehicles in the scenario (first cycle)
    resReselectionCounter(simValues.IDvehicle) = (simParams.minRandValueMode4-1) + randi((simParams.maxRandValueMode4-simParams.minRandValueMode4)+1,1,length(simValues.IDvehicle));
    % Set value 0 to vehicles that are blocked
    resReselectionCounter(BRid==-1)=0;
    
    % Initialization of sensing matrix (BRAlgorithm=8)
    sensingMatrix = zeros(simParams.NsensingPeriod,appParams.Nbeacons,simValues.maxID);
    knownUsedMatrix = zeros(appParams.Nbeacons,simValues.maxID);
end

% Initialization of lambda: SINR threshold for BRAlgorithm 9
if simParams.BRAlgorithm==9
    lambda = phyParams.gammaMin;
end

% Initialization of packet generation time
timeNextPacket = appParams.Tbeacon * rand(simValues.maxID,1);

% Initialization of time of last successfully sent packet.
lastSendTimeMatirx = 0;

%Initialization of age Histogram
res = [];
firstPacketTransmitted =  0;
HistogramMartix = zeros(16,20); %15*10meters =150 X 0.01-0.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation Cycle

% Number of snapshots
simValues.snapshots = ceil(simParams.simulationTime/appParams.Tbeacon);

% Start stopwatch
tic

fprintf('Simulation Time: ');
reverseStr = '';

for snap = 1:simValues.snapshots
    
    % Print time to video
    elapsedTime = round(snap*appParams.Tbeacon*100)/100;

    if snap==1
        % Print first cycle without estimation of end
        msg = sprintf('%.1f / %.1fs',elapsedTime,simParams.simulationTime);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
    else
        reverseStr = printUpdateToVideo(elapsedTime,simParams.simulationTime,reverseStr);
    end
    
    %% Position Update
    
    if mod(elapsedTime,simValues.timeResolution)==0 || snap==1
        % Update position only if timeElapsed is a multiple of timeResolution
        % or if it is the first cycle of simulation
        
        if ~simParams.fileTrace
            % Call function to update vehicles real positions
            [simValues.XvehicleReal,simValues.YvehicleReal,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePosition(simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,simValues.v,simValues.direction,appParams.Tbeacon,simValues.Xmax);
        else
            % If it is the first cycle of the simulation, keep the
            % positions at time zero, else read positions at elapsed time
            if snap==1 && simValues.timeResolution~=appParams.Tbeacon
                updateTime = 0;
            else
                updateTime = elapsedTime;
            end
            
            % Store IDs of vehicles at the previous beacon period and update positions
            [simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePositionFile(updateTime,simValues.dataTrace,simValues.IDvehicle);
            % Update BRid vector (variable number of vehicles in the scenario)
            [BRid] = updateBRidFile(BRid,simValues.IDvehicle,indexNewVehicles);
        end
        
        if simParams.BRAlgorithm==8
            % Update resReselectionCounter for vehicles exiting and entering the scenario (BRAlgorithm=8)
            % (vehicles that enter or are blocked start with a counter set to 0)
            resReselectionCounter(IDvehicleExit) = Inf;
            resReselectionCounter(BRid==-1) = 0;
        end
        
        if simParams.BRAlgorithm==3 || simParams.BRAlgorithm==4 || simParams.BRAlgorithm==5
            % No positioning error (AUTONOMOUS)
            simValues.Xvehicle = simValues.XvehicleReal;
            simValues.Yvehicle = simValues.YvehicleReal;
        else
            % Current position update period
            PosUpdatePeriod = mod(round(elapsedTime/appParams.Tbeacon)-1,NPosUpdate)+1;
            
            % Add LTE positioning delay (if selected)
            [simValues.Xvehicle,simValues.Yvehicle,PosUpdateIndex] = addPosDelay(simValues.Xvehicle,simValues.Yvehicle,simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,posUpdateAllVehicles,PosUpdatePeriod);
            
            % Add LTE positioning error (if selected)
            % (Xvehicle, Yvehicle): fictitious vehicles' position seen by the eNB
            [simValues.Xvehicle(PosUpdateIndex),simValues.Yvehicle(PosUpdateIndex)] = addPosError(simValues.XvehicleReal(PosUpdateIndex),simValues.YvehicleReal(PosUpdateIndex),simParams.sigmaPosError);
        end
        
        % Find vehicles not at the borders
        indexNoBorder = find((simValues.XvehicleReal(:,1)>=(simValues.Xmin+simParams.Mborder) & simValues.XvehicleReal(:,1)<=(simValues.Xmax-simParams.Mborder)) & (simValues.YvehicleReal(:,1)>=(simValues.Ymin+simParams.Mborder) & simValues.YvehicleReal(:,1)<=(simValues.Ymax-simParams.Mborder)));
        
        % Call function to compute distances
        % distance(i,j): distance from vehicle with index i to vehicle with index j
        [distanceReal,awarenessID,neighborsDistance,neighborsID,allNeighborsID] = computeDistance(simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,phyParams.Raw,phyParams.RawMax);
        if ~simParams.posError95 && NPosUpdate==1
            distance = distanceReal;
        else
            [distance,~,~,~,allNeighborsID] = computeDistance(simValues.Xvehicle,simValues.Yvehicle,simValues.IDvehicle,phyParams.Raw,phyParams.RawMax);
        end
        
        % Call function to update distance matrix where D(i,j) is the
        % change in distance of link i to j from time n-1 to time n and used
        % for updating Shadowing matrix
        [dUpdate,simValues.Shadowing_dB,distanceRealOld] = updateDistance(distanceReal,distanceRealOld,indexOldVehicles,indexOldVehiclesToOld,simValues.Shadowing_dB,phyParams.stdDevShadowLOS_dB);
        
        % Call function to compute received power
        % RXpower has dimensions IDvehicle X IDvehicle to speed up computation
        % RXpower(i,j): power received by vehicle with index i from vehicle with index j
        if ~phyParams.winnerModel
            [RXpower,CHgain,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpower(simValues.IDvehicle,distanceReal,phyParams.PtxERP_RB,phyParams.Gr,phyParams.L0,phyParams.beta,simValues.XvehicleReal,simValues.YvehicleReal,phyParams.Abuild,phyParams.Awall,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
        else
            [RXpower,CHgain,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpowerWinner(simValues.IDvehicle,distanceReal,phyParams.PtxERP_RB,phyParams.Gr,simValues.XvehicleReal,simValues.YvehicleReal,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
        end
        
        % Floor coordinates for PRRmap creation (if enabled)
        if outParams.printPRRmap
            simValues.XmapFloor = floor(simValues.Xmap);
            simValues.YmapFloor = floor(simValues.Ymap);
        end
    end
    
    % Call function to calculate effective neighbors (if enabled)
    if simParams.neighborsSelection
        [effAwarenessID,effNeighborsID,XvehicleRealOld,YvehicleRealOld,angleOld] = computeSignificantNeighbors(simValues.IDvehicle,simValues.XvehicleReal,simValues.YvehicleReal,XvehicleRealOld,YvehicleRealOld,neighborsID,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,angleOld,simParams.Mvicinity,phyParams.Raw,phyParams.RawMax,neighborsDistance);
    end
    
    %% Error Detection
    
    if ~outParams.printDistanceDetails && simParams.BRAlgorithm~=8
        
        % Create awareness BRid Matrix
        % Row index -> index of the vehicle
        % Column = BRIDs assigned to vehicles in the awareness range
        awarenessBRid = BRidmatrix(awarenessID,BRid);
        
        
        % Compute SINR of received beacons
        awarenessSINR = computeSINR(simValues.IDvehicle,BRid,appParams.NbeaconsT,appParams.NbeaconsF,awarenessID,RXpower,phyParams.IBEmatrix,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB);
        
        if simParams.neighborsSelection
            % Keep only significant neighbors (within Raw)
            awarenessID = effAwarenessID;
            awarenessBRid = (awarenessID>0).*awarenessBRid;
        end
        
        % Error detection (within Raw)
        % Create Error Matrix = [ID RX, ID TX, BRid, distance]
        [errorMatrix,resultsID,res,lastSendTimeMatrix,firstPacketTransmitted,HistogramMartix] = findErrors(simValues.IDvehicle,awarenessID,awarenessSINR,awarenessBRid,distanceReal,phyParams.gammaMin,elapsedtime,timeNextPacket,lastSendTimeMatirx,ageHist,res,firstPacketTransmitted,HistogramMartix);
        errorMatrixNoBorder = errorRemoveBorder(simValues.IDvehicle,errorMatrix,indexNoBorder);


       
        
    else
        
        % Create neighbors and awareness BRid Matrix
        neighborsBRid = BRidmatrix(neighborsID,BRid);
        awarenessBRid = (awarenessID>0).*neighborsBRid;
        
        % Compute SINR of received beacons
        neighborsSINR = computeSINR(simValues.IDvehicle,BRid,appParams.NbeaconsT,appParams.NbeaconsF,neighborsID,RXpower,phyParams.IBEmatrix,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB);
        
        if simParams.neighborsSelection
            % Keep only significant neighbors (up to RawMax)
            neighborsID = effNeighborsID;
            neighborsBRid = (neighborsID>0).*neighborsBRid;
            
            % Keep only significant neighbors (within Raw)
            awarenessID = effAwarenessID;
            awarenessBRid = (awarenessID>0).*awarenessBRid;
        end
        
        % Error detection (up to RawMax)
        [errorMatrixRawMax,resultsID,res,lastSendTimeMatrix,firstPacketTransmitted,HistogramMartix] = findErrors(simValues.IDvehicle,neighborsID,neighborsSINR,neighborsBRid,distanceReal,phyParams.gammaMin,elapsedTime,timeNextPacket,lastSendTimeMatirx,res,firstPacketTransmitted,HistogramMartix);
        errorMatrixRawMaxNoBorder = errorRemoveBorder(simValues.IDvehicle,errorMatrixRawMax,indexNoBorder);
        
        % Error detection (within Raw)
        errorMatrix = errorMatrixRawMax(errorMatrixRawMax(:,4)<phyParams.Raw,:);
        errorMatrixNoBorder = errorMatrixRawMaxNoBorder(errorMatrixRawMaxNoBorder(:,4)<phyParams.Raw,:);
        
    end
    
    % Check the correctness of SCI messages
    if simParams.BRAlgorithm==8
        errorSCImatrix = neighborsSINR < phyParams.minSCIsinr;
    end
    
    %% KPIs Computation (Snapshot)
    
    % Number of vehicles in the scenario (removing border effect)
    outputValues.NvehiclesNoBorder = length(indexNoBorder);
    outputValues.NvehiclesNoBorderTOT = outputValues.NvehiclesNoBorderTOT + outputValues.NvehiclesNoBorder;
    
    % Blocked vehicles (removing border effect)
    outputValues.NblockedNoBorder = nnz(BRid(simValues.IDvehicle(indexNoBorder),1)<0);
    outputValues.NblockedNoBorderTOT = outputValues.NblockedNoBorderTOT + outputValues.NblockedNoBorder;
    
    % Call function to create Awareness Matrix (with and removing border effect)
    % [#Correctly decoded beacons, #Errors, #Blocked neighbors, #Neighbors]
    awarenessMatrix = counter(simValues.IDvehicle,awarenessBRid,errorMatrix);
    awarenessMatrixNoBorder = awarenessMatrix(indexNoBorder,:);
    
    % Number of neighbors (removing border effect)
    outputValues.NneighboursNoBorder = sum(awarenessMatrixNoBorder(:,4));
    outputValues.NneighborsNoBorderTOT = outputValues.NneighborsNoBorderTOT + outputValues.NneighboursNoBorder;
    outputValues.StDevNeighboursNoBorder = std(awarenessMatrixNoBorder(:,4));
    outputValues.StDevNeighboursNoBorderTOT = outputValues.StDevNeighboursNoBorderTOT + outputValues.StDevNeighboursNoBorder;
    
    % Number of errors (removing border effect)
    outputValues.NerrorsNoBorder = length(errorMatrixNoBorder(:,1));
    outputValues.NerrorsNoBorderTOT = outputValues.NerrorsNoBorderTOT + outputValues.NerrorsNoBorder;
    
    % Number of received beacons (correct + errors) (removing border effect)
    outputValues.NreceivedBeaconsNoBorder = sum(awarenessMatrixNoBorder(:,1)) + sum(awarenessMatrixNoBorder(:,2));
    outputValues.NreceivedBeaconsNoBorderTOT = outputValues.NreceivedBeaconsNoBorderTOT + outputValues.NreceivedBeaconsNoBorder;
    
    % Number of correctly received beacons (removing border effect)
    outputValues.NcorrectlyReceivedBeaconsNoBorder = sum(awarenessMatrixNoBorder(:,1));
    outputValues.NcorrectlyReceivedBeaconsNoBorderTOT = outputValues.NcorrectlyReceivedBeaconsNoBorderTOT + outputValues.NcorrectlyReceivedBeaconsNoBorder;
    
    % Compute update delay (if enabled)
    if outParams.printUpdateDelay
        [simValues.updateTimeMatrix,outputValues.updateDelayCounter] = countUpdateDelay(simValues.IDvehicle,BRid,appParams.NbeaconsT,awarenessID,errorMatrix,elapsedTime,simValues.updateTimeMatrix,outputValues.updateDelayCounter,outParams.delayResolution);
    end
    
    % Compute packet delay (if enabled)
    if outParams.printPacketDelay
        outputValues.packetDelayCounter = countPacketDelay(simValues.IDvehicle,BRid,appParams.Tbeacon,appParams.NbeaconsT,elapsedTime,timeNextPacket,awarenessMatrix(:,4),errorMatrix(:,2),outputValues.packetDelayCounter,outParams.delayResolution);
    end
    
    % Compute power control allocation (if enabled)
    if outParams.printPowerControl
        % Convert linear PtxERP values to Ptx in dBm
        Ptx_dBm = 10*log10((phyParams.PtxERP_RB*appParams.RBsBeacon)/(2*phyParams.Gt))+30;
        outputValues.powerControlCounter = countPowerControl(BRid,Ptx_dBm,outputValues.powerControlCounter,outParams.powerResolution);
    end
    
    % Print number of neighbors per vehicle to file (if enabled)
    if outParams.printNeighbors
        printNeighborsToFile(awarenessMatrixNoBorder(:,4),outParams);
    end
    
    % Count details for distances up to the maximum awareness range (if enabled)
    if outParams.printDistanceDetails
        outputValues.distanceDetailsCounter = countDistanceDetails(neighborsDistance,neighborsBRid,errorMatrixRawMaxNoBorder,outputValues.distanceDetailsCounter,indexNoBorder);
    end
    
    % Update matrices needed for PRRmap creation in urban scenarios (if enabled)
    if outParams.printPRRmap
        simValues = counterMap(simValues,awarenessMatrix);
    end
    
    %% Update time next packet
    
    timeNextPacket = timeNextPacket + appParams.Tbeacon;
    
    %% Radio Resources Reassignment
    
    if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==6 || simParams.BRAlgorithm==7
        
        % Current scheduled reassign period
        reassignPeriod = mod(round(elapsedTime/appParams.Tbeacon)-1,NScheduledReassign)+1;
        
        % Find Index of vehicles which will perform sensing (BRAlgorithm=6)
        scheduledIndex = find(scheduledReassign(simValues.IDvehicle)==reassignPeriod);
        
        % Find IDs of vehicles whose resource will be reassigned
        scheduledID = simValues.IDvehicle(scheduledReassign(simValues.IDvehicle)==reassignPeriod);
        
    end
    
    if simParams.BRAlgorithm==1
        
        % BRs reassignment (CONTROLLED)
        % Call function for BRs reassignment
        % Returns updated BRid vector and number of successful reassignments
        alreadyReassigned = 0;
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentControlled(simValues.IDvehicle,alreadyReassigned,errorMatrix,distance,BRid,appParams.Nbeacons,indexNoBorder,phyParams.Rreuse,simParams.randomOrder);
        
    elseif simParams.BRAlgorithm==2
        
        % BRs reassignment (CONTROLLED with SCHEDULED REASSIGNMENT)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentControlledScheduled(simValues.IDvehicle,BRid,scheduledID,distance,errorMatrix,appParams.Nbeacons,indexNoBorder,phyParams.Rreuse,simParams.randomOrder);
        
    elseif simParams.BRAlgorithm==3
        
        % BRs reassignment (AUTONOMOUS with SENSING RANGE)
        % Vehicles that cannot decode beacons advise transmitting vehicles
        % Vehicles search for an available BR in their sensing range (defined by design)
        % Returns updated BRid vector and number of successful reassignments
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoSense(simValues.IDvehicle,errorMatrix,distanceReal,BRid,appParams.Nbeacons,indexNoBorder,phyParams.Rsense);
        
    elseif simParams.BRAlgorithm==4
        
        % BRs reassignment (AUTONOMOUS with MAP-RP)
        % Vehicles read BR maps (2 bits per BR -> free, busy, error)
        % Vehicles search for an available BR collecting BR maps
        % Returns updated BRid vector and number of successful reassignments
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoMap(simValues.IDvehicle,errorMatrix,awarenessID,awarenessBRid,BRid,appParams.Nbeacons,indexNoBorder);
        
    elseif simParams.BRAlgorithm==5
        
        % BRs reassignment (AUTONOMOUS with SENSING ALGORITHM BY QUALCOMM INC.)
        % Vehicles consider resource reselection according to the design
        % parameters of the sensing algorithm (p,k,M)
        % Vehicles which perform sensing do not transmit for a beacon
        % period (BRid = -3)
        [BRid,oldBRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoQC(simValues.IDvehicle,BRid,oldBRid,simParams.pReselect,simParams.kBest,simParams.hysteresysM,RXpower,appParams.Nbeacons,indexNoBorder,phyParams.PnRB);
        
    elseif simParams.BRAlgorithm==6
        
        % BRs reassignment (AUTONOMOUS with SENSING ALGORITHM BY INTEL CORP.)
        
        % Sensing-based BRs reassignment
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoIntel(simValues.IDvehicle,BRid,scheduledIndex,simParams.MBest,RXpower,appParams.Nbeacons,indexNoBorder,phyParams.PnRB);
        
    elseif simParams.BRAlgorithm==7
        
        % BRs reassignment (CONTROLLED with MAXIMUM REUSE DISTANCE)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder,BRidRT] = BRreassignmentControlledMaxReuse(simValues.IDvehicle,BRid,scheduledID,allNeighborsID,appParams.NbeaconsT,appParams.NbeaconsF,indexNoBorder,0,elapsedTime);
        %disp(BRidRT);
        %disp(BRid);
    elseif simParams.BRAlgorithm==8
        
        % BRs reassignment (3GPP MODE 4)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder,sensingMatrix,knownUsedMatrix,resReselectionCounter] = BRreassignment3GPPmode4(simValues.IDvehicle,BRid,sensingMatrix,knownUsedMatrix,resReselectionCounter,neighborsID,errorSCImatrix,simParams,timeNextPacket,RXpower,phyParams.IBEmatrix,appParams.Nbeacons,appParams.NbeaconsF,appParams.NbeaconsT,indexNoBorder,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB,appParams);
        
    elseif simParams.BRAlgorithm==9
        
        if mod(elapsedTime-appParams.Tbeacon,simParams.Treassign)==0
            
            % BRs reassignment (CONTROLLED with POWER CONTROL)
            [BRid,phyParams.PtxERP_RB,lambda,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentControlledPC(simValues.IDvehicle,BRid,phyParams.PtxERP_RB,CHgain,awarenessID,indexNoBorder,appParams.Nbeacons,lambda,phyParams.gammaMin,phyParams.PnRB,simParams.blockTarget,phyParams.maxPtxERP_RB);
            
        else
            NreassignNoBorder = 0;
            NunlockedNoBorder = 0;
        end
        
    elseif simParams.BRAlgorithm==101
        
        % Call Benchmark Algorithm 101 (RANDOM ALLOCATION)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentRandom(simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
        
    elseif simParams.BRAlgorithm==102
        
        % Call Benchmark Algorithm 102 (ORDERED ALLOCATION)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentOrdered(simValues.XvehicleReal,simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
        
    end
    
    % Successfully reassigned:
    % Stop the timer here to record the receiver time.
    
    
    
    % Incremental sum of successfully reassigned and unlocked vehicles
    outputValues.NreassignNoBorderTOT = outputValues.NreassignNoBorderTOT + NreassignNoBorder;
    outputValues.NunlockedNoBorderTOT = outputValues.NunlockedNoBorderTOT + NunlockedNoBorder;
    
end

%filename="OutputPeakValue.csv";
%csvwrite(filename,res);

filename = "HistogramMatrix_Benchmark_rho_300.csv";
csvwrite(filename,HistogramMartix);

% Stop stopwatch
outputValues.computationTime = toc;

end