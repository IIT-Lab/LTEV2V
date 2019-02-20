function [simParams,phyParams] = initiateBRAssignmentAlgorithm(simParams,phyParams,Tbeacon,fileCfg,varargin)
% function simParams = initiateBRAssignmentAlgorithm(simParams,fileCfg,varargin)
%
% Main settings of the simulation
% It takes in input the structure simParams, the name of the (possible) file config and the inputs
% of the main function
% It returns the structure "simParams"

fprintf('Settings of resource assignement algorithm\n');

% [BRAlgorithm]
% Selects the BR reassignment algorithm:
% 1 -> CONTROLLED
% 2 -> CONTROLLED with SCHEDULED BR REASSIGNMENT
% 3 -> AUTONOMOUS with SENSING RANGE
% 4 -> AUTONOMOUS with BR MAP
% 5 -> AUTONOMOUS with SENSING (QUALCOMM)
% 6 -> AUTONOMOUS with SENSING (INTEL)
% 7 -> CONTROLLED with MAXIMUM REUSE DISTANCE
% 8 -> AUTONOMOUS with SENSING (3GPP STANDARD MODE 4)
% 9 -> CONTROLLED with POWER CONTROL

% [BENCHMARKS Algorithms]
% Algorithms used as benchmarks
% 101 -> RANDOM ALLOCATION
% 102 -> ORDERED ALLOCATION (following X coordinate)

simParams = addNewParam(simParams,'BRAlgorithm',1,'Assignment algorithm','integer',fileCfg,varargin{1}{1});
if simParams.BRAlgorithm~=1 && simParams.BRAlgorithm~=2 && ...
        simParams.BRAlgorithm~=3 && simParams.BRAlgorithm~=4 && ...
        simParams.BRAlgorithm~=5 && simParams.BRAlgorithm~=6 && ...
        simParams.BRAlgorithm~=7 && simParams.BRAlgorithm~=8 && ...
        simParams.BRAlgorithm~=9 && ...
        simParams.BRAlgorithm~=101 && simParams.BRAlgorithm~=102
    error('Error: "simParams.BRAlgorithm" not valid. Algorithm not implemented.');
end

% [randomOrder]
% Selects if the resources is chosen randomly 
% (first assignment [all BRAlgorithm except 101,102,7] and for BRAlgorithm 1,2)
simParams = addNewParam(simParams,'randomOrder',true,'If the resources are selected randomly (first assignment)','bool',fileCfg,varargin{1}{1});

% [posError95]
% LTE Positioning Accuracy (Gaussian model): 95th percentile of the error (m)
simParams = addNewParam(simParams,'posError95',0,'LTE positioning error - 95th percentile (only controlled) (m)','double',fileCfg,varargin{1}{1});
simParams.sigmaPosError = simParams.posError95/1.96;   % Standard deviation of the error (m)

% [Tupdate]
% Time interval between each position update at the eNodeBs (s)
simParams = addNewParam(simParams,'Tupdate',Tbeacon,'Time interval between position updates at the eNodeBs (s)','double',fileCfg,varargin{1}{1});
if simParams.Tupdate<=0
    error('Error: "simParams.Tupdate" cannot <= 0');
end

if simParams.BRAlgorithm==1 || simParams.BRAlgorithm==2
    % [Mreuse]
    % Reuse margin (m) (only valid for controlled LTE-V2V with reuse distance [BRAlgorithm 1,2])
    simParams = addNewParam(simParams,'Mreuse',0,'Reuse margin (m)','integer',fileCfg,varargin{1}{1});
else
    simParams.Mreuse=0;
end

if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==6 || simParams.BRAlgorithm==7 || simParams.BRAlgorithm==9
    % [Treassign]
    % Time interval between each scheduled BR reassignment (BRAlgorithm 2,7,9) (s)
    % By default it is set equal to the beacon period
    simParams = addNewParam(simParams,'Treassign',Tbeacon,'Interval of scheduled reassignment (BRAlgorithm 2,7,9) (s)','double',fileCfg,varargin{1}{1});
    if simParams.Treassign<=0
        error('Error: "simParams.Treassign" cannot be <= 0.');
    end
end

if simParams.BRAlgorithm == 3
    % [Rsense]
    % Sensing Range (Autonomous case)
    phyParams = addNewParam(phyParams,'Rsense',phyParams.Raw,'Sensing range (m)','integer',fileCfg,varargin{1}{1});
end

if simParams.BRAlgorithm == 5
    % [pReselect]
    % Probability of resources reselection
    simParams = addNewParam(simParams,'pReselect',0.1,'Probability of resource reselection','double',fileCfg,varargin{1}{1});
    
    % [kBest]
    % Number of best candidates for resource reselection
    simParams = addNewParam(simParams,'kBest',20,'Number of best candidates for resource reselection','integer',fileCfg,varargin{1}{1});
    
    % [hysteresysM]
    % Hysteresys Margin (dB)
    simParams = addNewParam(simParams,'hysteresysM_dB',6,'Hysteresys Margin (dB) for resource reselection','double',fileCfg,varargin{1}{1});
    simParams.hysteresysM = 10^(simParams.hysteresysM_dB/10); 
end

if simParams.BRAlgorithm == 6
    % [Tsps]
    % Semi-persistent Scheduling -> Resource Reselection Period
    % (it must be a multiple of the beacon period)
    simParams = addNewParam(simParams,'Tsps',0.5,'Resource Reselection Period','double',fileCfg,varargin{1}{1});
    simParams.Treassign = simParams.Tsps;
    
    % [MBest]
    % Number of best candidates for resource reselection
    simParams = addNewParam(simParams,'MBest',20,'Number of best candidates for resource reselection','integer',fileCfg,varargin{1}{1});
end

if simParams.BRAlgorithm == 8   
    % [probResKeep]
    % Probability to keep the previously selected BR
    simParams = addNewParam(simParams,'probResKeep',0.8,'Probability to keep the previously selected BR','double',fileCfg,varargin{1}{1});
    if simParams.probResKeep<0 || simParams.probResKeep>0.8
        error('Error: "simParams.probResKeep" must be within 0 and 0.8');
    end
    
    % [ratioSelectedMode4]
    % Percentage of resources to be considered for random selection
    simParams = addNewParam(simParams,'ratioSelectedMode4',0.2,'Percentage of resources to be considered for random selection','double',fileCfg,varargin{1}{1});
    if simParams.ratioSelectedMode4<=0 || simParams.ratioSelectedMode4>1
        error('Error: "simParams.ratioSelectedMode4" must be more than 0 and not more than 1 (specs: 0.2)');
    end
    
    % [NsensingPeriod]
    % Number of beacon periods during which performing sensing
    simParams = addNewParam(simParams,'NsensingPeriod',10,'Number of beacon periods during which performing sensing','integer',fileCfg,varargin{1}{1});
    if simParams.NsensingPeriod<=0
        error('Error: "simParams.NsensingPeriod" must be larger than 0');
    end

    % [minRandValueMode4]
    % Minimum duration keeping the same allocation
    simParams = addNewParam(simParams,'minRandValueMode4',-1,'Minimum duration keeping the same allocation','integer',fileCfg,varargin{1}{1});
    if simParams.minRandValueMode4~=-1 && simParams.minRandValueMode4<=0
        error('Error: "simParams.minRandValueMode4" must be more than 0');
    end

    % [maxRandValueMode4]
    % Maximum duration keeping the same allocation
    simParams = addNewParam(simParams,'maxRandValueMode4',-1,'Maximum duration keeping the same allocation','integer',fileCfg,varargin{1}{1});
    if simParams.maxRandValueMode4~=-1 && simParams.maxRandValueMode4<=simParams.minRandValueMode4 
        error('Error: "simParams.maxRandValueMode4" must be larger than "simParams.minRandValueMode4"');
    end

    % [subframeT1Mode4]
    % Minimum subframe for the next allocation
    simParams = addNewParam(simParams,'subframeT1Mode4',1,'Minimum subframe for the next allocation in Mode 4','integer',fileCfg,varargin{1}{1});
    if simParams.subframeT1Mode4<1 || simParams.subframeT1Mode4>4
        error('Error: "simParams.subframeT1Mode4" must be between 1 and 4');
    end

    % [subframeT2Mode4]
    % Maximum subframe for the next allocation
    simParams = addNewParam(simParams,'subframeT2Mode4',100,'Maximum subframe for the next allocation in Mode 4','integer',fileCfg,varargin{1}{1});
    if simParams.subframeT2Mode4<20 || simParams.subframeT2Mode4>100
        error('Error: "simParams.subframeT2Mode4" must be between 20 and 100');
    end
    
    % [powerThresholdMode4]
    % Minimum power threshold to consider a BR as occupied in dBm
    simParams = addNewParam(simParams,'powerThresholdMode4',-110,'Minimum power threshold to consider a BR as occupied in Mode 4, in dBm','double',fileCfg,varargin{1}{1});
    if simParams.powerThresholdMode4/2<-64 || simParams.powerThresholdMode4/2>-1 || mod(simParams.powerThresholdMode4,2)~=0
        error('Error: "simParams.powerThresholdMode4" must be between -128 and -2, step 2 dB');
    end
    % From dBm to linear
    simParams.powerThresholdMode4 = 10^((simParams.powerThresholdMode4-30)/10);
     
    % [minSCIsinr]
    % Minimum SINR for a SCI to be correctly decoded
    phyParams = addNewParam(phyParams,'minSCIsinr',0,'Minimum SINR for a SCI to be correctly decoded, in dB','double',fileCfg,varargin{1}{1});
    phyParams.minSCIsinr = 10^((phyParams.minSCIsinr)/10);
end

if simParams.BRAlgorithm == 9
    % [blockTarget]
    % Target blocking rate
    simParams = addNewParam(simParams,'blockTarget',0.01,'Target blocking rate','double',fileCfg,varargin{1}{1});
end

if simParams.BRAlgorithm == 4
    error('Error: BRAlgorithm 4 not supported in the current version of LTEV2Vsim');
end

fprintf('\n');

end
