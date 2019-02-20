function Thr = throughputCalculator(Nbps,CR)
% This function calculates Throughput

Thr = (12*14*Nbps*CR)/(1e-3*180e3);

end