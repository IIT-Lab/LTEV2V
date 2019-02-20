function IBEmatrix = IBEcalculation(nRBperSubframeTot,nRBperSubframeToAlloc,nRBperBeacon,nBeaconPerSubframe,nRBperSubchannel,MCS,Ptx_dBm,ifAdjacent)

%%
% Setting of the parameters
% Parameters W,X,Y,Z
W = 3;
X = 6;
Y = 3;
Z = 3;
% EVM
if MCS >= 0 && MCS <= 10
    % BPSK-QPSK
    EVM = 0.175;
elseif MCS <= 20
    % 16-QAM
    EVM = 0.125;
else
    error('MCS in IBEcalculation() not valid. MCS must be in 0-20');
end
% N_RB is the total bandwidth
N_RB = nRBperSubframeTot;

% If nBeaconPerSubframe is 1, there cannot be IBE and the IBEmatrix is set to 1
% If nBeaconPerSubframe is 0, it means that more more than one subbrframe
% is needed to allocate a beacon; also in such case, IBEmatrix must be set to 1
if nBeaconPerSubframe==1 || nBeaconPerSubframe==0
    IBEmatrix = 1;
    return
end
%%

%%
% Initialization
IBEmatrix = ones(nBeaconPerSubframe,nBeaconPerSubframe);
startRB = -1*ones(1,nBeaconPerSubframe);
stopRB = -1*ones(1,nBeaconPerSubframe);

rbPerGap = ((nRBperSubchannel-nRBperBeacon/2)+2*(1-(~ifAdjacent)))*ones(1,nBeaconPerSubframe-1);
%%

% Setting the start and end of each beacon resource
%%%% START PLOT1
%%%figure(101);
%%%plot([1 nRBperSubframe],[1 1],'--r');
%%%hold on
%%%% END PLOT1
startRB(1) = 1;
stopRB(1) = nRBperBeacon/2;
%%%% START PLOT1
%%%plot(startRB(1):stopRB(1),ones(1,nRBperBeacon/2),'ok');
%%%% STOP PLOT1
for i=2:nBeaconPerSubframe
    startRB(i) = stopRB(i-1)+rbPerGap(i-1)+1;
    stopRB(i) = startRB(i)+nRBperBeacon/2-1;
    %%%% START PLOT1
    %%%plot(stopRB(i-1)+1:startRB(i)-1,zeros(1,rbPerGap(i-1)),'pb');
    %%%plot(startRB(i):stopRB(i),ones(1,nRBperBeacon/2),'ob');
    %%%% STOP PLOT1
end
%%

%%
% Calculating the IBE
%%%% START PLOT2
%%%int1Plot = -50*ones(1,nRBperSubframeToAlloc);
%%%int2Plot = -50*ones(1,nRBperSubframeToAlloc);
%%%% STOP PLOT2
for iBeacon1 = 1:nBeaconPerSubframe
    for iBeacon2 = 1:nBeaconPerSubframe
        % From interferer iBeacon2 to useful iBeacon1
        if iBeacon1~=iBeacon2
            % Transmitted power in one RB in dBm
            P_RB = 10^((Ptx_dBm-30)/10)/(stopRB(iBeacon2)-startRB(iBeacon2)+1);
            P_RB_dBm = 10*log10(P_RB)+30;
            % Setting the bandwidth of the interfering signal
            L_CRB = stopRB(iBeacon2)-startRB(iBeacon2)+1;            
            interference = 0;
            for rbIndex=startRB(iBeacon1):stopRB(iBeacon1) % in the interfered window
                %
                % 1) GENERAL PART
                % Setting the gap Delta_RB betweeen interfering and useful signals
                % if iBeacon2>iBeacon1:     Delta_RB = max(startRB(iBeacon2)-rbIndex
                % else:                     Delta_RB = rbIndex-stopRB(iBeacon2))
                % The same is obtained using a max() function
                Delta_RB = max(startRB(iBeacon2)-rbIndex, rbIndex-stopRB(iBeacon2));
                % Interference calculation
                interferenceG_dB = max(max( -25-10*log10(N_RB/L_CRB)-X,20*log10(EVM)-3-5*(abs(Delta_RB)-1)/L_CRB-W),(-57/180e3-P_RB_dBm-X)-30);
                interferenceG = 10^( interferenceG_dB/10 );
                %
                % 2) IQ IMAGE
                % Find the image of the rbIndex, looking at nRBperSubframeTot
                offsetRBtot = floor((nRBperSubframeTot-nRBperSubframeToAlloc)/2);
                rbImage = (nRBperSubframeTot-(rbIndex+offsetRBtot)+1) - offsetRBtot; 
                if rbImage>=startRB(iBeacon2) && rbImage<=stopRB(iBeacon2)
                    interferenceIQ = 10^( (-25-Y)/10 ); 
                else
                    interferenceIQ = 0;
                end
                %
                % 3) CARRIER LEACKAGE
                interferenceCL = 0;
                if mod(nRBperSubframeTot,2)==1 % ODD (TOT): one RB
                    if rbIndex==ceil(nRBperSubframeToAlloc/2)
                       interferenceCL = 10^( (-25-Z)/10 ); 
                    end  
                else % EVEN (TOT): two RBs
                    if rbIndex==ceil(nRBperSubframeToAlloc/2)-1 || rbIndex==ceil(nRBperSubframeToAlloc/2)
                       interferenceCL = 10^( (-25-Z)/10 ); 
                    end  
                end
                % OPTION1: Maximum between the sum and P_RB_dBm-30
                %interference = interference + max(10^((P_RB_dBm-30)/10),interferenceG+interferenceIQ+interferenceCL);
                % OPTION2: Directly the sum
                interference = interference + interferenceG+interferenceIQ+interferenceCL;
                %%%% START PLOT2
                %%%if iBeacon2==1
                %%%    int1Plot(rbIndex) = 10 * log10(interferenceG+interferenceIQ+interferenceCL);
                %%%end
                %%%if iBeacon2==2
                %%%    int2Plot(rbIndex) = 10 * log10(interferenceG+interferenceIQ+interferenceCL);
                %%%end
                %%%% STOP PLOT2
            end    
            % Average over the allocated bandwidth
            interference = interference/(stopRB(iBeacon1)-startRB(iBeacon1));
            IBEmatrix(iBeacon1,iBeacon2) = interference;
        end
    end
end
%%

%%%% START PLOT2
%%%figure(102);
%%%plot(1:nRBperSubframeToAlloc,int1Plot,'ok');
%%%hold on
%%%grid on
%%%plot(1:nRBperSubframeToAlloc,int2Plot,'pr');
%%%% STOP PLOT1



