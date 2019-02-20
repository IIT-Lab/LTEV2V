function phyParams = initiatePhyParameters(simParams,appParams,fileCfg,varargin)
% function phyParams = initiatePhyParameters(fileCfg,varargin)
%
% Settings of the PHY layer
% It takes in input the name of the (possible) file config and the inputs
% of the main function
% It returns the structure "phyParams"

fprintf('Physical layer settings\n');

%% Common PHY parameters

K = 1.38e-23;           % Avogadro's constant (J/K)
T0 = 290;               % Reference temperature (°K)

% [BwMHz]
% Bandwidth (MHz)
phyParams = addNewParam([],'BwMHz',10,'Bandwidth (MHz)','double',fileCfg,varargin{1}{1});
if phyParams.BwMHz~=1.4 && phyParams.BwMHz~=5 && phyParams.BwMHz~=10 && phyParams.BwMHz~=20
    error('Invalid Bandwidth. Possible values: 1.4, 5, 10, 20.');
end

% [Raw]
% Sets the awareness range in meters
phyParams = addNewParam(phyParams,'Raw',150,'Awareness range (m)','integer',fileCfg,varargin{1}{1});
if phyParams.Raw<=0
    error('Error: "appParams.Raw" cannot be <= 0');
end

% [Ptx_dBm]
% Transmitted power (dBm)
phyParams = addNewParam(phyParams,'Ptx_dBm',23,'Transmitted power (dBm)','double',fileCfg,varargin{1}{1});
phyParams.Ptx = 10^((phyParams.Ptx_dBm-30)/10);

% [Gt_dB]
% Transmitter antenna gain (dB)
phyParams = addNewParam(phyParams,'Gt_dB',3,'Transmitter antenna gain (dB)','double',fileCfg,varargin{1}{1});
phyParams.Gt = 10^(phyParams.Gt_dB/10);

% [PtxERP_dBm]
% Effective radiated power (dBm)
phyParams.PtxERP_dBm = phyParams.Ptx_dBm + phyParams.Gt_dB;
phyParams.PtxERP = 10^((phyParams.PtxERP_dBm-30)/10);

% [Gr_dB]
% Receiver antenna gain (dB)
phyParams = addNewParam(phyParams,'Gr_dB',3,'Receiver antenna gain (dB)','double',fileCfg,varargin{1}{1});
phyParams.Gr = 10^(phyParams.Gr_dB/10);

% [F_dB]
% Noise figure of the receiver (dB)
phyParams = addNewParam(phyParams,'F_dB',9,'Noise figure of the receiver (dB)','double',fileCfg,varargin{1}{1});

if ~simParams.useLTE
    
    %% PHY parameters related to 802.11p
    
    % [PnBW_dBm]
    % Noise power over the entire bandwidth (dBm)
    phyParams.PnBW_dBm = 10*log10(K*T0*phyParams.BwMHz*10e5) + phyParams.F_dB + 30;
    phyParams.PnBW = 10^((phyParams.PnBW_dBm-30)/10);
    
    % [Mode]
    % 802.11p TX Mode, from 1 to 8
    % Mode 3 is normally considered as the best trade-off
    phyParams = addNewParam(phyParams,'Mode',3,'TX Mode','integer',fileCfg,varargin{1}{1});
    if phyParams.Mode<1 || phyParams.Mode>8
        error('Error: "phyParams.Mode" must be within [1,8]');
    end
    
    % [CW]
    % Contention Window
    % Often assumed equal to 8; a better value in high dense scenarios might be 16
    phyParams = addNewParam(phyParams,'CW',16,'Contention Window','integer',fileCfg,varargin{1}{1});
    if phyParams.CW<1
        error('Error: "phyParams.CW" must be at least 1');
    end
    
    % PHY timing (not to be changed)
    
    % Arbitration Inter-frame Space (s)
    phyParams.tAifs = 58e-6;
    % Slot time (s)
    phyParams.tSlot = 13e-6;
    
    % Sensitivity when the preamble is not decoded (clear channel assessment, CCA, threshold)
    % In IEEE 802.11-2007, pag. 618, the value indicated is -65 dBm
    phyParams.PrxSensNotSynch = 10^((-65-30)/10);
    
    % Find minimum SINR
    phyParams.gammaMin = SINRmin11p(phyParams.Mode);
    phyParams.gammaMin_dB = 10*log10(phyParams.gammaMin);
    
    % Find the duration of a packet, derived from the packet payload and Mode
    phyParams.tPck = packetDuration11p(appParams.beaconSizeBytes,phyParams.Mode);
    
else
    
    %% PHY parameters related to LTE-V2V
    
    phyParams.Tfr = 0.01;                        % LTE frame period (s)
    phyParams.Tsf = phyParams.Tfr/10;            % LTE subframe period (s)
    phyParams.Tslot = phyParams.Tsf/2;           % LTE time slot period (s)
    
    % [PnRB_dBm]
    % Noise power over a RB (dBm)
    phyParams.RBbandwidth = 180e3;    % Bandwidth of a RB (Hz)
    phyParams.PnRB_dBm = 10*log10(K*T0*phyParams.RBbandwidth) + phyParams.F_dB + 30;
    phyParams.PnRB = 10^((phyParams.PnRB_dBm-30)/10);
    
    % [MCS]
    % Sets the modulation and coding scheme between 0 and 28
    phyParams = addNewParam(phyParams,'MCS',4,'Modulation and coding scheme','integer',fileCfg,varargin{1}{1});
    if phyParams.MCS<0 || phyParams.MCS>28
        error('Error: "phyParams.MCS" must be within [0,28]');
    end
    
    % [duplex]
    % Sets the duplexing: HD or FD
    phyParams = addNewParam(phyParams,'duplex','HD','Duplexing type','string',fileCfg,varargin{1}{1});
    if ~strcmp(phyParams.duplex,'HD') && ~strcmp(phyParams.duplex,'FD')
        error('Error: "phyParams.duplex" must be equal to HD (half duplex) or FD (full duplex)');
    end
    
    % [Ksi_dB]
    % Self-interference cancellation coefficient (dB)
    if strcmp(phyParams.duplex,'HD')
        phyParams.Ksi_dB = Inf;
        phyParams.Ksi = Inf;
    else
        phyParams = addNewParam(phyParams,'Ksi_dB',-110,'Self-interference cancellation coefficient (dB)','double',fileCfg,varargin{1}{1});
        phyParams.Ksi = 10^(phyParams.Ksi_dB/10);
    end
    
    % [NumBeaconsFrequency]
    % Specify the number of BRs in the frequency domain
    % -1 -> default value: exploitation of all the available BRs
    phyParams = addNewParam(phyParams,'NumBeaconsFrequency',-1,'Specify the number of BRs in the frequency domain','integer',fileCfg,varargin{1}{1});
    if phyParams.NumBeaconsFrequency<=0 && phyParams.NumBeaconsFrequency~=-1
        error('Error: "phyParams.NumBeaconsFrequency" must be -1 (all) or larger than 0');
    end
    
    % [ifAdjacent]
    % Select the subchannelization scheme: adjacent or non-adjacent PSCCH and PSSCH
    phyParams = addNewParam(phyParams,'ifAdjacent',true,'If using adjacent PSCCH and PSSCH','bool',fileCfg,varargin{1}{1});
    if phyParams.ifAdjacent < 0
        error('Error: "phyParams.ifAdjacent" must be equal to false or true');
    end
    
    % [sizeSubchannel]
    % Select the subchannel size according to 3GPP TS 36.331
    % -1 -> default value: automatically select the best value
    phyParams = addNewParam(phyParams,'sizeSubchannel',-1,'Subchannel size','integer',fileCfg,varargin{1}{1});
    if phyParams.sizeSubchannel<=0 && phyParams.sizeSubchannel~=-1
        error('Error: "phyParams.sizeSubchannel" must be -1 (best choice) or larger than 0');
    end
    
end

%% Channel Model parameters

% [winnerModel]
% Boolean to activate the use of WINNER+ B1 channel model (3GPP specifications)
phyParams = addNewParam(phyParams,'winnerModel',true,'If using Winner+ channel model','bool',fileCfg,varargin{1}{1});
if phyParams.winnerModel < 0
    error('Error: "phyParams.winnerModel" must be equal to false or true');
end

% [stdDevShadowLOS_dB]
% Standard deviation of shadowing in LOS (dB)
phyParams = addNewParam(phyParams,'stdDevShadowLOS_dB',3,'Standard deviation of shadowing in LOS (dB)','integer',fileCfg,varargin{1}{1});

% [stdDevShadowNLOS_dB]
% Standard deviation of shadowing in NLOS (dB)
phyParams = addNewParam(phyParams,'stdDevShadowNLOS_dB',4,'Standard deviation of shadowing in NLOS (dB)','integer',fileCfg,varargin{1}{1});

if ~phyParams.winnerModel
    
    % [L0]
    % Path loss at 1m (dB)
    phyParams = addNewParam(phyParams,'L0_dB',47.86,'Path loss at 1m (dB)','double',fileCfg,varargin{1}{1});
    phyParams.L0 = 10^(phyParams.L0_dB/10);
    
    % [beta]
    % Path loss exponent
    phyParams = addNewParam(phyParams,'beta',2.20,'Path loss exponent','double',fileCfg,varargin{1}{1});
    
    % [Abuild]
    % Attenuation every meter inside buildings (dB)
    phyParams = addNewParam(phyParams,'Abuild_dB',0.4,'Attenuation every meter inside buildings (dB)','double',fileCfg,varargin{1}{1});
    phyParams.Abuild = 10^(phyParams.Abuild_dB/10);
    
    % [Awall]
    % Attenuation for each wall crossed (dB)
    phyParams = addNewParam(phyParams,'Awall_dB',6,'Attenuation for each wall crossed (dB)','double',fileCfg,varargin{1}{1});
    phyParams.Awall = 10^(phyParams.Awall_dB/10);
    
end

fprintf('\n');

end
