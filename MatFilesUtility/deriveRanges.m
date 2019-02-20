function [phyParams] = deriveRanges(phyParams,simParams)
% Derive maximum awareness range and other ranges according to the selected algorithm

if phyParams.winnerModel
    % Winner Model Parameters
    h = 1.5;                            % Antenna height [m]
    h_eff = h - 1;                      % Effective antenna height [m]
    f = 5.9;                            % Central frequency [GHz]
    
    L0WinnerLOS_dB = 7.56-34.6*log10(h_eff)+2.7*log10(f);
    L0WinnerLOS = 10^(L0WinnerLOS_dB/10);
    L0WinnerNLOS_dB = 5.83*log10(h)+18.38+23*log10(f);
    
    if ~simParams.useLTE
        
        % When using 802.11p
        
        % Compute maximum awareness range in LOS (m) (Winner Model)
        a = - L0WinnerLOS_dB + phyParams.Gr_dB + phyParams.PtxERP_dBm - phyParams.gammaMin_dB - phyParams.PnBW_dBm;
        phyParams.RawMaxLOS = 10^(a/40);
        % Compute maximum awareness range in NLOS (m) (Winner Model)
        b = (- L0WinnerNLOS_dB + phyParams.Gr_dB + phyParams.PtxERP_dBm - phyParams.gammaMin_dB - phyParams.PnBW_dBm)/(44.9-6.55*log10(h));
        phyParams.RawMaxNLOS = 10^b;
        
    else
        
        % When using LTE-V2V
        
        % Compute maximum awareness range in LOS (m) (Winner Model)
        a = - L0WinnerLOS_dB + phyParams.Gr_dB + phyParams.PtxERP_RB_dBm - phyParams.gammaMin_dB - phyParams.PnRB_dBm;
        phyParams.RawMaxLOS = 10^(a/40);
        % Compute maximum awareness range in NLOS (m) (Winner Model)
        b = (- L0WinnerNLOS_dB + phyParams.Gr_dB + phyParams.PtxERP_RB_dBm - phyParams.gammaMin_dB - phyParams.PnRB_dBm)/(44.9-6.55*log10(h));
        phyParams.RawMaxNLOS = 10^b;
        
    end
    
    % Compute maximum range with 2 times standard deviation of shadowing in LOS (m)
    a2sigma = a + 2*phyParams.stdDevShadowLOS_dB;
    phyParams.RawMax =  10^(a2sigma/40);
    
else
    
    % Convert 2 times st. dev. of shadowing from dB to linear
    Shadow2SigmaLOS = 10^((2*phyParams.stdDevShadowLOS_dB)/10);
    
    if ~simParams.useLTE
        
        % When using 802.11p
        
        % Compute maximum awareness range in LOS (m)
        phyParams.RawMaxLOS = ((phyParams.PtxERP*phyParams.Gr)/(phyParams.gammaMin*phyParams.L0*phyParams.PnBW))^(1/phyParams.beta);
        
        % Compute maximum range with 2 times standard deviation of shadowing in LOS (m)
        phyParams.RawMax = ((phyParams.PtxERP*phyParams.Gr*Shadow2SigmaLOS)/(phyParams.gammaMin*phyParams.L0*phyParams.PnBW))^(1/phyParams.beta);
        
    else
        
        % When using LTE-V2V
        
        % Compute maximum awareness range in LOS (m)
        phyParams.RawMaxLOS = ((phyParams.PtxERP_RB*phyParams.Gr)/(phyParams.gammaMin*phyParams.L0*phyParams.PnRB))^(1/phyParams.beta);
        
        % Compute maximum range with 2 times standard deviation of shadowing in LOS (m)
        phyParams.RawMax = ((phyParams.PtxERP_RB*phyParams.Gr*Shadow2SigmaLOS)/(phyParams.gammaMin*phyParams.L0*phyParams.PnRB))^(1/phyParams.beta);
        
    end
    
end

if phyParams.Raw > phyParams.RawMax
    fprintf('The awareness range exceeds the maximum possible one: ');
    phyParams.Raw = phyParams.RawMax;
    fprintf('set to %.0f m\n\n', phyParams.Raw);
end

if simParams.useLTE
    
    if simParams.BRAlgorithm~=101 || simParams.BRAlgorithm~=102 || simParams.BRAlgorithm~=7
        
        if phyParams.winnerModel
            
            % Compute minimum reuse distance (m) (Winner Model)
            Rreuse1 = phyParams.Raw + phyParams.Raw/(((1/phyParams.gammaMin)-(phyParams.PnRB/phyParams.PtxERP_RB)*(L0WinnerLOS*phyParams.Raw^4)/phyParams.Gr)^(1/4));
            
        else
            
            % Compute minimum reuse distance (m)
            Rreuse1 = phyParams.Raw + phyParams.Raw/(((1/phyParams.gammaMin)-(phyParams.PnRB/phyParams.PtxERP_RB)*(phyParams.L0*phyParams.Raw^phyParams.beta)/phyParams.Gr)^(1/phyParams.beta));
            
        end
        
        Rreuse2 = 2*phyParams.Raw;
        
        RreuseMin = max([Rreuse1 Rreuse2]);
        
        % Reuse distance (m)
        phyParams.Rreuse = RreuseMin + simParams.Mreuse;
        
    end
    
    if simParams.BRAlgorithm==3
        
        if phyParams.winnerModel
            
            % Compute maximum sensing range (Rnoise) (m)
            phyParams.Rnoise = ((phyParams.PtxERP_RB*phyParams.Gr)/(L0WinnerLOS*phyParams.PnRB))^(1/4);
            if phyParams.Rsense<phyParams.Raw || phyParams.Rsense>phyParams.Rnoise
                error('The sensing range is over the limits: should be between %.0f m and %.0f m\n',phyParams.Raw,phyParams.Rnoise);
            end
            
        else
            
            % Compute maximum sensing range (Rnoise) (m)
            phyParams.Rnoise = ((phyParams.PtxERP_RB*phyParams.Gr)/(phyParams.L0*phyParams.PnRB))^(1/phyParams.beta);
            if phyParams.Rsense<phyParams.Raw || phyParams.Rsense>phyParams.Rnoise
                error('The sensing range is over the limits: should be between %.0f m and %.0f m\n',phyParams.Raw,phyParams.Rnoise);
            end
            
        end
        
    end
    
end

end
