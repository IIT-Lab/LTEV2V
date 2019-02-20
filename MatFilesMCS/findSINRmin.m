function gammaMin_dB = findSINRmin(Thr)
% This function calculates the minimum SINR 
% (alfa is taken from 3GPP documents)

alfa = 0.4;
gammaMin_dB = 10*log10(2^(Thr/alfa)-1);

end