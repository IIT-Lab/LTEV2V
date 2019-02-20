function packetDelayCounter = countPacketDelay(IDvehicle,BRid,Tbeacon,NbeaconsT,elapsedTime,timeNextPacket,Nneighbors,errorsTx,packetDelayCounter,delayResolution)
% Function to compute the packet delay between received beacons
% Returns the updated packetDelayCounter

delayMax = length(packetDelayCounter)*delayResolution;

% Calculate BRidT = vector of BRid in the time domain
BRidT = mod(BRid-1,NbeaconsT)+1;

% Find vehicles that are currently transmitting (not blocked)
indexTx = find(BRid(IDvehicle)>0);

for i = 1:length(indexTx)
    % Find number of neighbors of the Tx vehicle
    iNneighbors = Nneighbors(indexTx(i));
    % Find number of errors caused by the Tx vehicle
    iNerrors = nnz(errorsTx==IDvehicle(indexTx(i)));
    % Number of correct receptions
    iNcorrect = iNneighbors - iNerrors;
    % Compute packet delay of the Tx vehicle (module is used to adapt the
    % calculation to the simulator design)
    packetDelay = mod(elapsedTime + (0.1/NbeaconsT)*(BRidT(IDvehicle(indexTx(i)))) - timeNextPacket(IDvehicle(indexTx(i))),Tbeacon);
    if packetDelay>=delayMax
        % Increment last counter
        packetDelayCounter(end) = packetDelayCounter(end) + iNcorrect;
    else
        % Increment counter corresponding to the current delay
        packetDelayCounter(ceil(packetDelay/delayResolution)) = ...
            packetDelayCounter(ceil(packetDelay/delayResolution)) + iNcorrect;
    end
end

end
