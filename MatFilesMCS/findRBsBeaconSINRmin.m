function [RBsBeacon,gammaMin_dB] = findRBsBeaconSINRmin(MCS,BeaconSizeBits)
% This function calculates RBs per beacon and minimum required SINR

% Call function to find ITBS value from MCS
ITBS = findITBS(MCS);

% Call function to find the modulation format (number of bits per symbol)
Nbps = findModulation(MCS);

% Call function to find the number of RBs per beacon
[RBsBeacon,Nbits] = findRBsBeaconNbits(ITBS,BeaconSizeBits);

% Compute the effective code rate
CR = findCR(Nbits,RBsBeacon,Nbps);

% Compute throughput
Thr = throughputCalculator(Nbps,CR);

% Compute the minimum required SINR
gammaMin_dB = findSINRmin(Thr);

end