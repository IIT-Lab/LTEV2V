function [simValues,outputValues,appParams,simParams,phyParams,outParams] = main11p(appParams,simParams,phyParams,outParams,simValues,outputValues)
% The function main11p() performs the IEEE 802.11p simulation

%% Initialization

% Initialize Tx power per RB vector
phyParams.maxPtxERP = phyParams.PtxERP; % Max PtxERP
phyParams.PtxERP = phyParams.PtxERP*ones(simValues.maxID,1);

% Local vector of IDs
IDvehicle = simValues.IDvehicle;

% Number of vehicles at the current time
NvNow = length(IDvehicle);

% Copy real coordinates (no positioning error)
simValues.Xvehicle = simValues.XvehicleReal;
simValues.Yvehicle = simValues.YvehicleReal;

% Number of packets in the queue of each node
nPackets = zeros(simValues.maxID,1);

% State of each node
% The possible states are four:
% 1 = IDLE :    the node has no packet and senses the medium as free
% 2 = BACKOFF : the node has a packet to transmit and senses the medium as
%               free; it is thus performing the backoff
% 3 = TX :      the node is transmitting
% 9 = RX :      the node is sensing the medium as busy and possibly receiving
%               a packet (the sender it firstly sensed is saved in
%               idFromWhichRx)
vState = ones(simValues.maxID,1);

% When a node senses the medium as busy and goes to State RX, the
% transmitting node is saved in 'idFromWhichRx'
% Note that once a node starts receiving a signal, it will not be able to
% synchronize to a different signal, thus there is no reason to change this
% value before exiting from State RX
% 'idFromWhichRx' is set to the id of the node if the node is not receiving
% (a number must be set in order to avoid exceptions running the code that follow)
idFromWhichRx = (1:simValues.maxID)';

% Possible events: A) New packet, B) Backoff ends, C) Transmission end;
% A - 'timeNextPacket' stores the instant of the next message generation; the
% first instant is randomly chosen within 0-Tbeacon
timeNextPacket = Inf * ones(simValues.maxID,1);
timeNextPacket(IDvehicle) = appParams.Tbeacon * rand(NvNow,1);
timeNextGeneration = timeNextPacket;

% B+C - 'timeNextTxRx' stores the instant of the next backoff or
% transmission end, if any
timeNextTxRx = Inf * ones(simValues.maxID,1);

% A+B+C - stores the instant of the next event among all possible events;
% which event is deducted comparing this variable to those for A and B+C
timeNextEvent = Inf * ones(simValues.maxID,1);
timeNextEvent(IDvehicle) = timeNextPacket(IDvehicle);

% Total power being received from nodes in State 3
rxPowerTotLast = zeros(simValues.maxID,1);
rxPowerUsefulLast = zeros(simValues.maxID,1);

% Instant when the power store in 'PrTot' was calculated; it will remain
% constant until a new calculation will be performed
instantThisPrStarted = Inf;

% Average SINR - This parameter is irrelevant if the node is not in State 9
sinrAverage = zeros(simValues.maxID,1);

% Instant when the average SINR of a node in State 9 was initiated - This
% parameter is irrelevant if the node is not in State 9
instantThisSINRavStarted = Inf * ones(simValues.maxID,1);

% Number of slots for the backoff - Set to '-1' when not initiated
nSlotBackoff = -1 * ones(simValues.maxID,1);

% Find vehicles not at the borders
indexNoBorder = find((simValues.Xvehicle(:,1)>=(simValues.Xmin+simParams.Mborder) & simValues.Xvehicle(:,1)<=(simValues.Xmax-simParams.Mborder)) & (simValues.Yvehicle(:,1)>=(simValues.Ymin+simParams.Mborder) & simValues.Yvehicle(:,1)<=(simValues.Ymax-simParams.Mborder)));

% Call function to compute distances
% distance(i,j): distance from vehicle with index i to vehicle with index j
[distance,awarenessID,~,neighborsID] = computeDistance(simValues.Xvehicle,simValues.Yvehicle,IDvehicle,phyParams.Raw,phyParams.RawMax);

% Save distance matrix
distanceOld = distance;

% Initialize array of old of coordinates
XvehicleOld = simValues.Xvehicle;
YvehicleOld =  simValues.Yvehicle;

% Initialize array of old angles
angleOld = zeros(length(XvehicleOld),1);

% The power received by each node from each node is stored in the matrix 'RXpower'
% Call function to compute received power
% RXpower(i,j): power received by vehicle with index i from vehicle with index j
dUpdate = zeros(NvNow,NvNow);
if ~phyParams.winnerModel
    [RXpower,~,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpower(IDvehicle,distance,phyParams.PtxERP,phyParams.Gr,phyParams.L0,phyParams.beta,simValues.Xvehicle,simValues.Yvehicle,phyParams.Abuild,phyParams.Awall,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
else
    [RXpower,~,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpowerWinner(IDvehicle,distance,phyParams.PtxERP,phyParams.Gr,simValues.Xvehicle,simValues.Yvehicle,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
end

% Floor coordinates for PRRmap creation (if enabled)
if outParams.printPRRmap
    simValues.XmapFloor = floor(simValues.Xmap);
    simValues.YmapFloor = floor(simValues.Ymap);
end

% The simulation starts at time '0'
timeNow = 0;

% The variable 'timeNextPrint' is used only for printing purposes
timeNextPrint = 0;

% The variable 'timeNextPosUpdate' is used for updating the positions
timeNextPosUpdate = simValues.timeResolution;
numPosUpdates = 1;

% Number of vehicles in the scenario (removing border effect)
outputValues.NvehiclesNoBorder = length(indexNoBorder);
outputValues.NvehiclesNoBorderTOT = outputValues.NvehiclesNoBorderTOT + outputValues.NvehiclesNoBorder;

% Number of neighbors (removing border effect)
NneighborsRaw = zeros(length(indexNoBorder),1);
for i = 1:length(indexNoBorder)
    NneighborsRaw(i) = nnz(awarenessID(indexNoBorder(i),:));
end
outputValues.NneighboursNoBorder = sum(NneighborsRaw);
outputValues.NneighborsNoBorderTOT = outputValues.NneighborsNoBorderTOT + outputValues.NneighboursNoBorder;
outputValues.StDevNeighboursNoBorder = std(NneighborsRaw);
outputValues.StDevNeighboursNoBorderTOT = outputValues.StDevNeighboursNoBorderTOT + outputValues.StDevNeighboursNoBorder;

% Prepare matrix for update delay computation (if enabled)
if outParams.printUpdateDelay
    % Reset update time of vehicles that are outside the scenario
    allIDOut = setdiff(1:simValues.maxID,IDvehicle);
    simValues.updateTimeMatrix(allIDOut,:) = -1;
    simValues.updateTimeMatrix(:,allIDOut) = -1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation Cycle
% The simulation ends when the time exceeds the duration of the simulation
% (not really used, since a break inside the cycle will stop the simulation
% earlier)

% Start stopwatch
tic

fprintf('Simulation Time: ');
reverseStr = '';

while timeNow < simParams.simulationTime
    
    % The instant and node of the next event is obtained
    % indexEvent is the index of the vector IDvehicle
    % idEvent is the ID of the vehicle of the current event
    [timeEvent, indexEvent] = min(timeNextEvent(IDvehicle));
    idEvent = IDvehicle(indexEvent);
    
    % If timeEvent surpasses the next position update, set the time equal
    % to the position update
    if timeEvent>=timeNextPosUpdate
        timeEvent = timeNextPosUpdate;
    end
    
    % The time instant is updated
    % If the time instant exceeds the duration of the simulation, the
    % simulation is ended
    timeNow = timeEvent;
    if timeNow>simParams.simulationTime
        break;
    end
    
    % Print time to video
    while timeNow>timeNextPrint
        reverseStr = printUpdateToVideo(timeNow,simParams.simulationTime,reverseStr);
        timeNextPrint = timeNextPrint + appParams.Tbeacon;
    end
    
    %% Action
    % The action at thisinstant depends on the selected event
    
    % POSITION UPDATE: positions of vehicles are updated
    if timeEvent==timeNextPosUpdate
        %fprintf('Position update\n');
        
        if ~simParams.fileTrace
            % Call function to update vehicles positions
            [simValues.Xvehicle,simValues.Yvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePosition(simValues.Xvehicle,simValues.Yvehicle,IDvehicle,simValues.v,simValues.direction,appParams.Tbeacon,simValues.Xmax);
        else
            % Store IDs of vehicles at the previous beacon period and update positions
            [simValues.Xvehicle,simValues.Yvehicle,IDvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePositionFile(round(timeNextPosUpdate*100)/100,simValues.dataTrace,IDvehicle);
        end
        
        % Update value of next position update
        timeNextPosUpdate = timeNextPosUpdate + simValues.timeResolution;
        numPosUpdates = numPosUpdates+1;
        
        % Find vehicles not at the borders
        indexNoBorder = find((simValues.Xvehicle(:,1)>=(simValues.Xmin+simParams.Mborder) & simValues.Xvehicle(:,1)<=(simValues.Xmax-simParams.Mborder)) & (simValues.Yvehicle(:,1)>=(simValues.Ymin+simParams.Mborder) & simValues.Yvehicle(:,1)<=(simValues.Ymax-simParams.Mborder)));
        
        % Call function to compute distances
        % distance(i,j): distance from vehicle with index i to vehicle with index j
        [distance,awarenessID,neighborsDistance,neighborsID] = computeDistance(simValues.Xvehicle,simValues.Yvehicle,IDvehicle,phyParams.Raw,phyParams.RawMax);
        
        % Call function to update distance matrix where D(i,j) is the
        % change in distance of link i to j from time n-1 to time n and used
        % for updating Shadowing matrix
        [dUpdate,simValues.Shadowing_dB,distanceOld] = updateDistance(distance,distanceOld,indexOldVehicles,indexOldVehiclesToOld,simValues.Shadowing_dB,phyParams.stdDevShadowLOS_dB);
                
        % Call function to compute received power
        % RXpower(i,j): power received by vehicle with index i from vehicle with index j
        if ~phyParams.winnerModel
            [RXpower,~,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpower(IDvehicle,distance,phyParams.PtxERP,phyParams.Gr,phyParams.L0,phyParams.beta,simValues.Xvehicle,simValues.Yvehicle,phyParams.Abuild,phyParams.Awall,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
        else
            [RXpower,~,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpowerWinner(IDvehicle,distance,phyParams.PtxERP,phyParams.Gr,simValues.Xvehicle,simValues.Yvehicle,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
        end
        
        % Floor coordinates for PRRmap creation (if enabled)
        if outParams.printPRRmap
            simValues.XmapFloor = floor(simValues.Xmap);
            simValues.YmapFloor = floor(simValues.Ymap);
        end
        
        % Call function to calculate effective neighbors (if enabled)
        if simParams.neighborsSelection
            [effAwarenessID,effNeighborsID,XvehicleOld,YvehicleOld,angleOld] = computeSignificantNeighbors(IDvehicle,simValues.Xvehicle,simValues.Yvehicle,XvehicleOld,YvehicleOld,neighborsID,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,angleOld,simParams.Mvicinity,phyParams.Raw,phyParams.RawMax,neighborsDistance);
            awarenessID = effAwarenessID;
            neighborsID = effNeighborsID;
        end
        
        % Number of vehicles in the scenario (removing border effect)
        outputValues.NvehiclesNoBorder = length(indexNoBorder);
        outputValues.NvehiclesNoBorderTOT = outputValues.NvehiclesNoBorderTOT + outputValues.NvehiclesNoBorder;
        
        % Number of neighbors (removing border effect)
        NneighborsRaw = zeros(length(indexNoBorder),1);
        for i = 1:length(indexNoBorder)
            NneighborsRaw(i) = nnz(awarenessID(indexNoBorder(i),:));
        end
        outputValues.NneighboursNoBorder = sum(NneighborsRaw);
        outputValues.NneighborsNoBorderTOT = outputValues.NneighborsNoBorderTOT + outputValues.NneighboursNoBorder;
        outputValues.StDevNeighboursNoBorder = std(NneighborsRaw);
        outputValues.StDevNeighboursNoBorderTOT = outputValues.StDevNeighboursNoBorderTOT + outputValues.StDevNeighboursNoBorder;
        
        % Print number of neighbors per vehicle to file (if enabled)
        if outParams.printNeighbors
            printNeighborsToFile(NneighborsRaw,outParams);
        end
             
        % Prepare matrix for update delay computation (if enabled)
        if outParams.printUpdateDelay
            % Reset update time of vehicles that are outside the scenario
            allIDOut = setdiff(1:simValues.maxID,IDvehicle);
            simValues.updateTimeMatrix(allIDOut,:) = -1;
            simValues.updateTimeMatrix(:,allIDOut) = -1;
        end
        
        % Generate time values of new vehicles entering the scenario
        timeNextPacket(IDvehicle(indexNewVehicles)) = timeNow + appParams.Tbeacon * rand(1,length(indexNewVehicles));
        
        % End transmission of vehicles that are no more in the scenario
        if ~isempty(IDvehicleExit)
            % Reset time next packet and tx-rx for vehicles that exit the scenario
            timeNextPacket(IDvehicleExit) = Inf;
            timeNextTxRx(IDvehicleExit) = Inf;
            nPackets(IDvehicleExit) = zeros(length(IDvehicleExit),1);
            vState(IDvehicleExit) = ones(length(IDvehicleExit),1);
            idFromWhichRx(IDvehicleExit) = IDvehicleExit;
            instantThisSINRavStarted(IDvehicleExit) = Inf;
            
            % The average SINR of all vehicles is then updated
            sinrAverage = updateSINR11p(timeNow,IDvehicle,vState,idFromWhichRx,rxPowerTotLast,rxPowerUsefulLast,instantThisPrStarted,sinrAverage,instantThisSINRavStarted,phyParams.PnBW);
            
            % The nodes that may stop receiving must be checked
            [vState,idFromWhichRx,nSlotBackoff,timeNextTxRx] = checkVehiclesStopReceiving11p(timeNow,IDvehicle,vState,idFromWhichRx,nSlotBackoff,nPackets,timeNextTxRx,RXpower,phyParams);
            
            % The present overall/useful power received and the instant of calculation are updated
            % The power received must be calculated after
            % 'checkVehiclesStopReceiving11p', to have the correct idFromWhichtransmitting
            [instantThisPrStarted,rxPowerTotLast,rxPowerUsefulLast] = updateLastPower11p(timeNow,IDvehicle,vState,idFromWhichRx,RXpower,simValues);
        end
        
        % Reset time next packet and tx-rx for vehicles that exit the scenario
        timeNextPacket(IDvehicleExit) = Inf;
        timeNextTxRx(IDvehicleExit) = Inf;
        
        % CASE A: new packet is generated
    elseif timeEvent == timeNextPacket(idEvent)
        %fprintf('New packet\n');
        % The queue is updated
        % If one packet is already enqueued, the old packet is removed, the
        % number of errors is updated, and the number of packets discarded
        % is increased by one
        nPackets(idEvent) = nPackets(idEvent)+1;
        
        if nPackets(idEvent)>1
            % Count as a blocked transmission (previous packet is discarded)
            if isempty(find(indexNoBorder==indexEvent,1))==0
                outputValues.NblockedNoBorderTOT = outputValues.NblockedNoBorderTOT+1;
            end
            if outParams.printDistanceDetails
                for iRaw = 1:floor(phyParams.RawMax)
                    outputValues.distanceDetailsCounter(iRaw,4) = outputValues.distanceDetailsCounter(iRaw,4) + nnz(distance(indexNoBorder,indexEvent)<iRaw);
                end
            end
            nPackets(idEvent) = nPackets(idEvent)-1;
            %fprintf('CAM message discarded\n');
        end
        
        % If the node was in IDLE (State==1), the backoff is started
        if vState(idEvent)==1 % idle
            % Start the backoff (State==2)
            vState(idEvent)=2; % backoff
            % A new random backoff is set and the instant of its conclusion
            % is derived
            [nSlotBackoff(idEvent), timeNextTxRx(idEvent)] = startNewBackoff11p(timeNow,phyParams.CW,phyParams.tAifs,phyParams.tSlot);
        end
        
        % The time of the next packet generation is updated to now + the
        % beacon period + little fluctuation to avoid correlation in collisions
        timeNextGeneration(idEvent) = timeNextPacket(idEvent);
        timeNextPacket(idEvent) = timeNow + appParams.Tbeacon * (0.95 + 0.1 * rand(1));
        
        % CASE B+C: either a backoff or a transmission concludes
    else % txrxevent
        % A backoff ends
        if vState(idEvent)==2 % END backoff
            %fprintf('Backoff concluded\n');
            % A transmission starts:
            % - The backoff counter is reset
            % - The end of the transmission is set
            vState(idEvent) = 3; % tx
            nSlotBackoff(idEvent) = -1;
            timeNextTxRx(idEvent) = timeNow + phyParams.tPck;
            
            % The average SINR is updated
            sinrAverage = updateSINR11p(timeNow,IDvehicle,vState,idFromWhichRx,rxPowerTotLast,rxPowerUsefulLast,instantThisPrStarted,sinrAverage,instantThisSINRavStarted,phyParams.PnBW);
            
            % Check the nodes that start receiving
            [vState,idFromWhichRx,nSlotBackoff,sinrAverage,instantThisSINRavStarted,timeNextTxRx] = checkVehiclesStartReceiving11p(timeNow,idEvent,indexEvent,IDvehicle,vState,idFromWhichRx,nSlotBackoff,sinrAverage,instantThisSINRavStarted,timeNextTxRx,RXpower,phyParams);
            
            % The present overall/useful power received and the instant of calculation are updated
            % The power received must be calculated after
            % 'checkVehiclesStartReceiving11p', to have the correct idFromWhichtransmitting
            [instantThisPrStarted,rxPowerTotLast,rxPowerUsefulLast] = updateLastPower11p(timeNow,IDvehicle,vState,idFromWhichRx,RXpower,simValues);
            
            % A transmission ends
        elseif vState(idEvent)==3 % END tx
            %fprintf('Tx concluded\n');
            % The transmitting vehicle is first updated
            [vState,idFromWhichRx,nPackets,sinrAverage,instantThisSINRavStarted,timeNextTxRx,nSlotBackoff] = updateVehicleEndingTx11p(timeNow,idEvent,indexEvent,IDvehicle,vState,idFromWhichRx,nPackets,sinrAverage,instantThisSINRavStarted,timeNextTxRx,nSlotBackoff,RXpower,phyParams);
            
            % The average SINR of all vehicles is then updated
            sinrAverage = updateSINR11p(timeNow,IDvehicle,vState,idFromWhichRx,rxPowerTotLast,rxPowerUsefulLast,instantThisPrStarted,sinrAverage,instantThisSINRavStarted,phyParams.PnBW);

            % Update KPIs
            [simValues,outputValues] = updateKPI11p(timeNow,idEvent,indexEvent,IDvehicle,awarenessID(indexEvent,:)',neighborsID(indexEvent,:)',indexNoBorder,vState,idFromWhichRx,sinrAverage,distance,timeNextGeneration,phyParams,outParams,simValues,outputValues,simParams.neighborsSelection);

            % The nodes that may stop receiving must be checked
            [vState,idFromWhichRx,nSlotBackoff,timeNextTxRx] = checkVehiclesStopReceiving11p(timeNow,IDvehicle,vState,idFromWhichRx,nSlotBackoff,nPackets,timeNextTxRx,RXpower,phyParams);
            
            % The present overall/useful power received and the instant of calculation are updated
            % The power received must be calculated after
            % 'checkVehiclesStopReceiving11p', to have the correct idFromWhichtransmitting
            [instantThisPrStarted,rxPowerTotLast,rxPowerUsefulLast] = updateLastPower11p(timeNow,IDvehicle,vState,idFromWhichRx,RXpower,simValues);
        else
            fprintf('idEvent=%d, state=%d\n',idEvent,vState(idEvent));
            error('Ends unknown event...')
        end
    end
    
    % The next event is selected as the minimum of all values in 'timeNextPacket'
    % and 'timeNextTxRx'
    timeNextEvent = min(timeNextPacket,timeNextTxRx);
    if min(timeNextEvent(IDvehicle))<timeNow % error check
        fprintf('next=%f, now=%f\n',min(timeNextEvent(IDvehicle)),timeNow);
        error('An event is schedule in the past...');
    end
    
end

simValues.snapshots = numPosUpdates;

% Stop stopwatch
outputValues.computationTime = toc;

end
