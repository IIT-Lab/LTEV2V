function [Shadowing_dB] = computeShadowing(Shadowing_dB,LOS,dUpdate,stdDevShadowLOS_dB,stdDevShadowNLOS_dB,D_corr)
% Function that computes correlated shadowing samples w.r.t. the previous
% time instant

Nv = length(dUpdate(:,1));                           % Number of vehicles

% Generation of new samples of shadowing
newShadowing_dB = randn(Nv,Nv).*(LOS*stdDevShadowLOS_dB + (~LOS)*(stdDevShadowNLOS_dB));

% Calculation of correlated shadowing matrix
A = exp(-dUpdate/D_corr).*Shadowing_dB + sqrt( 1-exp(-2*dUpdate/D_corr) ).*newShadowing_dB;
Shadowing_dB = triu(A,1)+triu(A)';

end

