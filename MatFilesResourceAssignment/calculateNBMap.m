function [NewNbeaconsF,NewNbeaconsT,Nbeacons,NewNsubframesPerBeacon,NewBeaconSizeBits,NewRBsBeacon,NewgammaMin_dB] = calculateNBMap(RBsSubframeBeaconing,RBsBeacon,Tbc,Tsf,beaconSizeBits,MCS)
% Compute number of beacons per beacon period [AUTONOMOUS CASE MAP-RP]
% Iterative function

[~,~,NB1,~] = calculateNB(RBsSubframeBeaconing,RBsBeacon,Tbc,Tsf);

NB2 = 0;

while NB2 <= NB1
    
    % New beacon size (bits) = info bits + MAP-RP bits
    Bbits = beaconSizeBits + NB1*2;
    
    [RBsBeacon,gammaMin_dB] = findRBsBeaconSINRmin(MCS,Bbits);
    
    [NbeaconsF,NbeaconsT,NB2,NsubframesPerBeacon] = calculateNB(RBsSubframeBeaconing,RBsBeacon,Tbc,Tsf);
    
    if NB2 <= NB1
        Nbeacons = NB2;
        NewBeaconSizeBits = Bbits;
        NewNbeaconsF = NbeaconsF;
        NewNbeaconsT = NbeaconsT;
        NewNsubframesPerBeacon = NsubframesPerBeacon;
        NewRBsBeacon = RBsBeacon;
        NewgammaMin_dB = gammaMin_dB;
    end
    
    NB1 = NB1-1;
    
end

end