function [simValues,outputValues] = updateKPI11p(timeNow,idEvent,indexEvent,IDvehicle,awarenessID,neighborsID,indexNoBorder,vState,idFromWhichRx,sinrAverage,distance,timeNextGeneration,phyParams,outParams,simValues,outputValues,neighborsSelection)
% KPIs: correct transmissions and errors are counted
% The message is correctly received if:
% 1) the node is currently receiving
% 2) the node is receiving from idEvent
% 3) the average SINR is above the threshold
% Values are counted within a circle of radius raw
rxOK = (vState(IDvehicle)==9) .* (idFromWhichRx(IDvehicle)==idEvent) .* (sinrAverage(IDvehicle)>=phyParams.gammaMin);
rxOKNoBorder = (vState(IDvehicle(indexNoBorder))==9) .* (idFromWhichRx(IDvehicle(indexNoBorder))==idEvent) .* (sinrAverage(IDvehicle(indexNoBorder))>=phyParams.gammaMin);

% Number of correctly received beacons (removing border effect)
if ~neighborsSelection
    neighborsRaw = (IDvehicle~=idEvent) .* (distance(:,indexEvent)<phyParams.Raw);
    neighborsRawNoBorder = (IDvehicle(indexNoBorder)~=idEvent) .* (distance(indexNoBorder,indexEvent)<phyParams.Raw);
else
    neighborsRaw = ismember(IDvehicle,awarenessID);
    neighborsRawNoBorder = ismember(IDvehicle(indexNoBorder),awarenessID);
end
NneighborsRaw = nnz(neighborsRaw);
rxOKRaw = logical(neighborsRaw .* rxOK);
rxOKRawNoBorder = neighborsRawNoBorder .* (rxOKNoBorder);
NcorrectlyReceivedBeaconsNoBorder = nnz(rxOKRawNoBorder);
outputValues.NcorrectlyReceivedBeaconsNoBorderTOT = outputValues.NcorrectlyReceivedBeaconsNoBorderTOT + NcorrectlyReceivedBeaconsNoBorder;

% Number of errors (removing border effect)
NerrorsNoBorder = nnz(neighborsRawNoBorder .* (~rxOKNoBorder));
outputValues.NerrorsNoBorderTOT = outputValues.NerrorsNoBorderTOT + NerrorsNoBorder;

% Number of received beacons (correct + errors) (removing border effect)
outputValues.NreceivedBeaconsNoBorder = NcorrectlyReceivedBeaconsNoBorder + NerrorsNoBorder;
outputValues.NreceivedBeaconsNoBorderTOT = outputValues.NreceivedBeaconsNoBorderTOT + outputValues.NreceivedBeaconsNoBorder;

% Count correct receptions and errors up to the maximum awareness range (if enabled)
% (removing border effect)
if outParams.printDistanceDetails
    if ~neighborsSelection
        AllNeighborsNoBorder = (IDvehicle(indexNoBorder)~=idEvent);
    else
        AllNeighborsNoBorder = ismember(IDvehicle(indexNoBorder),neighborsID);
    end
    for iRaw = 1:floor(phyParams.RawMax)
        % Correctly decoded beacons
        RxOKiRaw = AllNeighborsNoBorder .* (distance(indexNoBorder,indexEvent)<iRaw) .* (rxOKNoBorder);
        outputValues.distanceDetailsCounter(iRaw,2) = outputValues.distanceDetailsCounter(iRaw,2) + nnz(RxOKiRaw);
        % Errors
        RxErroriRaw = AllNeighborsNoBorder .* (distance(indexNoBorder,indexEvent)<iRaw) .* (~rxOKNoBorder);
        outputValues.distanceDetailsCounter(iRaw,3) = outputValues.distanceDetailsCounter(iRaw,3) + nnz(RxErroriRaw);
    end
end

% Update matrices needed for PRRmap creation (if enabled)
if outParams.printPRRmap
    indexRxOkRaw = find(rxOKRaw);
    % Count correctly received beacons
    for i = 1:length(indexRxOkRaw)
        simValues.correctlyReceivedMap(simValues.YmapFloor(indexRxOkRaw(i)),simValues.XmapFloor(indexRxOkRaw(i))) = ...
            simValues.correctlyReceivedMap(simValues.YmapFloor(indexRxOkRaw(i)),simValues.XmapFloor(indexRxOkRaw(i))) + 1;
    end
    % Count neighbors of idEvent
    simValues.neighborsMap(simValues.YmapFloor(indexEvent),simValues.XmapFloor(indexEvent)) = ...
        simValues.neighborsMap(simValues.YmapFloor(indexEvent),simValues.XmapFloor(indexEvent)) + NneighborsRaw;
end

% Compute update delay (if enabled)
if outParams.printUpdateDelay
    % Find maximum delay in updateDelayCounter
    delayMax = length(outputValues.updateDelayCounter)*outParams.delayResolution;
    % Vehicles inside the awareness range of vehicle with ID i
    IDIn = awarenessID(awarenessID>0);
    % ID of vehicles that are outside the awareness range of vehicle i
    all = (1:length(simValues.updateTimeMatrix(:,1)))';
    IDOut = setdiff(all,IDIn);
    simValues.updateTimeMatrix(idEvent,IDOut)=-1;
    for i = 1:length(IDIn)
        % If the beacon is currently correctly received by the neighbor
        % inside the awareness range
        if find(IDvehicle(rxOKRaw)==IDIn(i))
            % Store previous timestamp
            previousTimeStamp = simValues.updateTimeMatrix(idEvent,IDIn(i));
            % If there was a previous timestamp
            if previousTimeStamp>0
                % Compute update delay
                updateDelay = timeNow - previousTimeStamp;
                if updateDelay>=delayMax
                    % Increment last counter
                    outputValues.updateDelayCounter(end) = outputValues.updateDelayCounter(end) + 1;
                else
                    % Increment counter corresponding to the current delay
                    outputValues.updateDelayCounter(ceil(updateDelay/outParams.delayResolution)) = ...
                        outputValues.updateDelayCounter(ceil(updateDelay/outParams.delayResolution)) + 1;
                end
            end
        end
    end
    % Update updateTimeMatrix with timeNow
    simValues.updateTimeMatrix(idEvent,IDvehicle(rxOKRaw)) = timeNow;
end

% Compute packet delay (if enabled)
if outParams.printPacketDelay
    % Find maximum delay in updateDelayCounter
    delayMax = length(outputValues.packetDelayCounter)*outParams.delayResolution;
    % Find number of correct receptions within phyParams.Raw
    NcorrectlyReceivedBeacons = nnz(rxOKRaw);
    % Compute packet delay
    packetDelay = timeNow - timeNextGeneration(idEvent);
    if packetDelay>=delayMax
        % Increment last counter
        outputValues.packetDelayCounter(end) = outputValues.packetDelayCounter(end) + NcorrectlyReceivedBeacons;
    else
        % Increment counter corresponding to the current delay
        outputValues.packetDelayCounter(ceil(packetDelay/outParams.delayResolution)) = ...
            outputValues.packetDelayCounter(ceil(packetDelay/outParams.delayResolution)) + NcorrectlyReceivedBeacons;
    end
end

end