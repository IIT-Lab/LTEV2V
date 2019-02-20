function sinrAverage = updateSINR11p(timeNow,IDvehicle,vState,idFromWhichRx,PrTotLast,PrUsefulLast,instantThisPrStarted,sinrAverage,instantThisSINRavStarted,PN)
% The average SINR is updatedUpdate as the weighted average
% between (i) the SINR calculated from 'instantThisSINRavStarted'
% and 'instantThisPrStarted', presently saved in 'sinrAverage'
% and (ii) the SINR calculated since the last Pr calculation,
% i.e., from 'instantThisPrStarted', to now
% The useful power is calculated as the power received from the
% node stored in 'idFromWhichRx'
% The interfering power is calculated as the overall power
% received from the nodes with State==3 minus the useful power

% VERSION 1: using matrixes
%PrUsefulLast = Pr( sub2ind(size(Pr),(1:nV)',idFromWhichRx') )';% selects useful Pr
%snrLast = PrUsefulLast ./ (PN + (PrTot-PrUsefulLast));
%sinrAverage = (sinrAverage .* (instantThisPrStarted-instantThisSINRavStarted) + ... 
%    snrLast .* (timeNow-instantThisPrStarted)) ./ (timeNow-instantThisSINRavStarted);

% VERSION 2: limited for cycle
for iVehicle = IDvehicle(vState(IDvehicle)==9)'
    if idFromWhichRx(iVehicle)==iVehicle
        sinrAverage(iVehicle) = 0;
    else
        sinrLast = PrUsefulLast(iVehicle) / (PN + (PrTotLast(iVehicle)-PrUsefulLast(iVehicle)));
        if sinrLast<0
            error('sinrLast of vehicle=%d (state=%d), receiving from=%d (state=%d) < 0',iVehicle,vState(iVehicle),idFromWhichRx(iVehicle),vState(idFromWhichRx(iVehicle)));
        end
        sinrAverage(iVehicle) = (sinrAverage(iVehicle) .* (instantThisPrStarted-instantThisSINRavStarted(iVehicle)) + ...
            sinrLast .* (timeNow-instantThisPrStarted)) ./ (timeNow-instantThisSINRavStarted(iVehicle));
    end
end            
% Tests have revealed that VERSION 2 reduces simulation duration