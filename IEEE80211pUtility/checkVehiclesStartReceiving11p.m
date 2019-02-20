function [vState,idFromWhichRx,nSlotBackoff,sinrAverage,instantThisSINRavStarted,timeNextTxRx] = checkVehiclesStartReceiving11p(timeNow,idEvent,indexEvent,IDvehicle,vState,idFromWhichRx,nSlotBackoff,sinrAverage,instantThisSINRavStarted,timeNextTxRx,rxPower,phyParams)
% The nodes that start receiving are identified
% They are those that:
% 1.  are in idle or in backoff (do not transmit and are not
% already receiving)
% 2. do not end the backoff in the next time slot (a 1e-10
% margin is added due to problems with the representation of
% real numbers)
% 3. receive this signal with sufficient quality (= are able to
% decode the preamble, since SINR>SINR_min) OR do not receive the
% signal with sufficient quality, but perceive the channel as
% busy
rxPowerTotNow = rxPower * (vState(IDvehicle)==3);
A = ( (vState(IDvehicle)==1) + (vState(IDvehicle)==2) );
B = (timeNextTxRx(IDvehicle) >= (timeNow+phyParams.tSlot-1e-10));
C = ( (rxPower(:,indexEvent) ./ (phyParams.PnBW + (rxPowerTotNow-rxPower(:,indexEvent))) > phyParams.gammaMin)...
    + (rxPowerTotNow >= phyParams.PrxSensNotSynch));

ifStartReceiving = logical( A .* ...
    B .* ...
    C );

% Focusing on those that start receiving
% The backoff is freezed if the node was in vState==2
% State is set to 9
% The node from which receiving is set
% SINR is reset and initial instant is set to now
% 'timeNextTxRx' is set to infinity
for iVehicle = IDvehicle(logical(ifStartReceiving.*(vState(IDvehicle)==2)))
    nSlotBackoff(iVehicle) = freezeBackoff11p(timeNow,timeNextTxRx(iVehicle),phyParams.tSlot,nSlotBackoff(iVehicle));
end
vState(IDvehicle(ifStartReceiving)) = 9;
idFromWhichRx(IDvehicle(ifStartReceiving)) = idEvent;
sinrAverage(IDvehicle(ifStartReceiving)) = 0;
instantThisSINRavStarted(IDvehicle(ifStartReceiving)) = timeNow;
timeNextTxRx(IDvehicle(ifStartReceiving)) = Inf;
