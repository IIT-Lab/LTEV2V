function gammaMin = SINRmin11p(Mode)
% IEEE 802.11p: find minimum SINR which depends on the operating Mode
% Values are derived from IEEE 802.11-2007, section 17.3.10.1

if Mode==1
    gammaMin = 10^((10)/10);
elseif Mode==2
    gammaMin = 10^((11)/10);
elseif Mode==3
    gammaMin = 10^((13)/10);
elseif Mode==4
    gammaMin = 10^((15)/10);
elseif Mode==5
    gammaMin = 10^((18)/10);
elseif Mode==6
    gammaMin = 10^((22)/10);
elseif Mode==7
    gammaMin = 10^((26)/10);
elseif Mode==8
    gammaMin = 10^((27)/10);
end

end
