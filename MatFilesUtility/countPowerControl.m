function powerControlCounter = countPowerControl(BRid,Ptx_dBm,powerControlCounter,powerResolution)
% Function to compute the power control allocation at each transmission
% Returns the updated powerControlCounter

% Exclude vehicles that are blocked or outside the scenario
Ptx_dBm = Ptx_dBm(BRid>0);

% Convert power to powerControlCounter vector
Ptx_dBm = round(Ptx_dBm/powerResolution)+101;
maxPtx = length(powerControlCounter);

for i = 1:length(Ptx_dBm)
    if Ptx_dBm(i)>=maxPtx
        powerControlCounter(end) = powerControlCounter(end) + 1;
    elseif Ptx_dBm(i)<=1
        powerControlCounter(1) = powerControlCounter(1) + 1;
    else
        powerControlCounter(Ptx_dBm(i)) = powerControlCounter(Ptx_dBm(i)) + 1;
    end  
end

end
