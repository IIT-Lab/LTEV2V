function [instantThisPrStarted,rxPowerTotLast,rxPowerUsefulLast] = updateLastPower11p(timeNow,IDvehicle,vState,idFromWhichRx,rxPower,simValues)
% rxPowerTotLast, rxPowerUsefulLast, and instantThisPrStarted are updated
% They will be used to compute the average SINR when the next transmission
% will start or end

rxPowerUsefulLast = zeros(simValues.maxID,1);
rxPowerTotLast = zeros(simValues.maxID,1);

rxPowerTotLast(IDvehicle) = rxPower * (vState(IDvehicle)==3);
for indexV = 1:length(IDvehicle)
    % the index of the node from which this node is receiving is computed
    indexFromWhichRx = find(IDvehicle==idFromWhichRx(IDvehicle(indexV)),1);
    % if indexFromWhichRx is empty, something is wrong: the vehicle from
    % which this node is receiving is not in the scenario
    if isempty(indexFromWhichRx)
        error('is empty indexFromWhichRx, IDvehicle(indexV)=%d',IDvehicle(indexV));
    end
    % the useful rx power is calculated
    rxPowerUsefulLast(IDvehicle(indexV)) = rxPower(indexV,indexFromWhichRx);
    % if the useful rx power is larger then the total rx power there is
    % something wrong, unless the node is just senisng the medium as busy; in such case, it should 
    % be receiving by 'itself'
    if rxPowerUsefulLast(IDvehicle(indexV)) > rxPowerTotLast(IDvehicle(indexV)) && ...
            idFromWhichRx(IDvehicle(indexV))~=IDvehicle(indexV)
        error('useful rx power > total rx power, vehicle=%d (state"%d) rx from=%d (state=%d)',IDvehicle(indexV),vState(IDvehicle(indexV)),idFromWhichRx(IDvehicle(indexV)),vState(idFromWhichRx(IDvehicle(indexV))));
    end
end
instantThisPrStarted = timeNow;