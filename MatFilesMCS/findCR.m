function CR = findCR(Nbits,RBsBeacon,Nbps)
% This function calculates the effective Code Rate

CR = Nbits/((RBsBeacon/2)*9*12*Nbps);

end