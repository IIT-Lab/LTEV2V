function [appParams,phyParams] = deriveBeaconResources(appParams,phyParams)

% Call function to find the total number of RBs in the frequency domain per Tslot
phyParams.RBsFrequency = RBtable(phyParams.BwMHz);

% Number of RBs allocated to the Cooperative Awareness (V2V)
appParams.RBsFrequencyV2V = floor(phyParams.RBsFrequency*(appParams.resourcesV2V/100));

% % Find the total number of RBs per subframe
phyParams.RBsSubframe = phyParams.RBsFrequency*(phyParams.Tsf/phyParams.Tslot);

% % Number of RBs allocated to the Cooperative Awareness
appParams.RBsSubframeBeaconing = appParams.RBsFrequencyV2V*(phyParams.Tsf/phyParams.Tslot);

% Call function to find the number of RBs needed to carry a beacon and minimum SINR
appParams.beaconSizeBits = appParams.beaconSizeBytes*8;
[appParams.RBsBeacon, phyParams.gammaMin_dB] = findRBsBeaconSINRmin(phyParams.MCS,appParams.beaconSizeBits);
phyParams.gammaMin = 10^(phyParams.gammaMin_dB/10);

% Check whether the beacon size (appParams.RBsBeacon) + SCI (2x 2 RBs) exceeds the number of available RBs per subframe
appParams.RBsSubframeV2V = appParams.RBsFrequencyV2V*(phyParams.Tsf/phyParams.Tslot);
if (appParams.RBsBeacon+4)>appParams.RBsSubframeV2V
    error('Error: "appParams.beaconSizeBytes" is too large for the selected MCS (packet cannot fit in a subframe)');
end

% Find NbeaconsF and NbeaconsT, subchannel sizes or multiples
[appParams,phyParams] = calculateNB(appParams,phyParams);

% When using BrAlgorithm 4 (MAP-RP) - currently not supported
%if simParams.BRAlgorithm==4   
%     % Call iterative function to compute new beacon size and number of beacons using MAP-RP
%     [NbeaconsF,NbeaconsT,Nbeacons,NsubframesPerBeacon,beaconSizeBits,appParams.RBsBeacon,gammaMin_dB] = calculateNBMap(RBsSubframeBeaconing,appParams.RBsBeacon,appParams.Tbeacon,phyParams.Tsf,beaconSizeBits,phyParams.MCS);
%error('Error: BRAlgorithm 4 is not supported in this version of LTEV2Vsim');
%end

% Compute radiated power per RB
phyParams.PtxERP_RB = phyParams.PtxERP/(appParams.RBsBeacon/2);
phyParams.PtxERP_RB_dBm = 10*log10(phyParams.PtxERP_RB)+30;

% Compute In-Band Emission Matrix (following 3GPP TS 36.101 V15.0.0)
phyParams.IBEmatrix = IBEcalculation(phyParams.RBsSubframe,appParams.RBsSubframeBeaconing,appParams.RBsBeacon,appParams.NbeaconsF,phyParams.RBsBeaconSubchannel,phyParams.MCS,phyParams.PtxERP_dBm,phyParams.ifAdjacent);

% Check how many BRs to exploit in the frequency domain
if phyParams.NumBeaconsFrequency~=-1
    if phyParams.NumBeaconsFrequency > appParams.NbeaconsF
        fprintf('Number of beacons in frequency domain in input is larger than the maximum one: set to %.0f\n\n', NbeaconsF);
    else
        appParams.NbeaconsF = phyParams.NumBeaconsFrequency;
        phyParams.IBEmatrix = phyParams.IBEmatrix(1:appParams.NbeaconsF,1:appParams.NbeaconsF);
    end
end

% Total number of beacons per beacon period = Beacon Resources (BRs)
appParams.Nbeacons = appParams.NbeaconsF*appParams.NbeaconsT;

% Error check
if appParams.Nbeacons<1
    fprintf('Number of beacons equal to %d. Error.', Nbeacons);
    error('');
end

end