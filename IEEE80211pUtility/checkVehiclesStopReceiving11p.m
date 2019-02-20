function [vState,idFromWhichRx,nSlotBackoff,timeNextTxRx] = checkVehiclesStopReceiving11p(timeNow,IDvehicle,vState,idFromWhichRx,nSlotBackoff,nPackets,timeNextTxRx,rxPower,phyParams)
% The nodes that may stop receiving must be checked
% Firstly, those nodes that were receiving from this node should change the
% receiving node to own, and then possibly stop receiving
% They are those that:
% 1) are currently receiving
% 2) do not have the 'idFromWhichRx' currently transmitting
ifReceivingFromThis = logical( (vState(IDvehicle)==9) .* ...
    (vState(idFromWhichRx(IDvehicle))~=3) );
idFromWhichRx(IDvehicle(ifReceivingFromThis)) = IDvehicle(ifReceivingFromThis);
% Then, those that also sense the medium as idle will exit from state 9
% 3) sense a total power below the threshold
rxPowerTotNow = rxPower * (vState(IDvehicle)==3);
ifStopReceiving = ifReceivingFromThis .* ...
    (rxPowerTotNow < phyParams.PrxSensNotSynch);
% Focusing on those that stop receiving
% The 'idFromWhichRx' is reset to the own id
% State is set to either 0 or 1 depending on whether the queue
% is empty or not
% If the queue is not empty, the backoff is started; if
% 'nSlotBackoff' contains a '-1', it means that a new backoff
% should be started; otherwise, it was freezed and should be
% resumed
vState(IDvehicle((logical(ifStopReceiving .* (nPackets(IDvehicle)==0))))) = 1;
vState(IDvehicle((logical(ifStopReceiving .* (nPackets(IDvehicle)>0))))) = 2;
for iVehicle = IDvehicle(logical(ifStopReceiving .* (nPackets(IDvehicle)>0)))
    if nSlotBackoff(iVehicle)==-1
        [nSlotBackoff(iVehicle), timeNextTxRx(iVehicle)] = startNewBackoff11p(timeNow,phyParams.CW,phyParams.tAifs,phyParams.tSlot);
    else
        timeNextTxRx(iVehicle) = resumeBackoff11p(timeNow,nSlotBackoff(iVehicle),phyParams.tAifs,phyParams.tSlot);
    end
end
