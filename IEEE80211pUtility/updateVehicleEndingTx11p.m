function [vState,idFromWhichRx,nPackets,sinrAverage,instantThisSINRavStarted,timeNextTxRx,nSlotBackoff] = updateVehicleEndingTx11p(timeNow,idEvent,indexEvent,IDvehicle,vState,idFromWhichRx,nPackets,sinrAverage,instantThisSINRavStarted,timeNextTxRx,nSlotBackoff,rxPower,phyParams)
% If the vehicle is exiting the scenario, indexEvent is set to -1
% thus shouldn't call this function
if indexEvent<=0
    error('Call to updateVehicleEndingTx11p with indexEvent<=0');
end

% The number of packets in the queue is reduced
nPackets(idEvent) = nPackets(idEvent)-1;
% The medium is sensed to check if it is free
% (note: 'vState(idEvent)' is set to 9 in order not to contribute
% to the sensed power)
vState(idEvent)=9; % rx
if ((vState(IDvehicle)==3) * rxPower(indexEvent,:)) > phyParams.PrxSensNotSynch
    % If it is not free, State 9 with error is entered
    vState(idEvent)=9; % rx
    idFromWhichRx(idEvent) = idEvent;
    sinrAverage(idEvent) = 0;
    instantThisSINRavStarted(idEvent) = timeNow;
    timeNextTxRx(idEvent) = Inf;
else
    % If it is not busy, then: the idle state is entered if the queue is empty
    % otherwise a new backoff is started
    if nPackets(idEvent)==0
        % If no other packets are in the queue, the node goes
        % in idle state
        vState(idEvent)=1; % idle
        timeNextTxRx(idEvent) = Inf;
    elseif nPackets(idEvent)>=1
        % If there are other packets in the queue, a new
        % backoff is initialized and started
        vState(idEvent)=2; % backoff
        [nSlotBackoff(idEvent), timeNextTxRx(idEvent)] = startNewBackoff11p(timeNow,phyParams.CW,phyParams.tAifs,phyParams.tSlot);
    else
        error('Error: nPackets<0');
    end
end

