function [RXpower,CHgain,Shadowing_dB,X,Y]  = computeRXpowerWinner(IDvehicle,distance,PtxERP_RB,Gr,Xvehicle,Yvehicle,XminMap,YmaxMap,StepMap,...
    GridMap,fileObstaclesMap,Shadowing_dB,dUpdate,stdDevShadowLOS_dB,stdDevShadowNLOS_dB)
% Compute received power and create RXpower matrix using Winner+ (B1) channel model
% [Formula corresponds to the one used in "deriveRanges.m" - Any changes have to be
% made also there]

% Description of WINNER+ (B1) channel model for path loss:

% LOS 1) distance lower than breakpoint distance(Dbp)
%        PL = 22.7log10(d) + 27.0 + 20log10(f)
%     2) distance higher than Dbp
%        PL = 40.0log10(d)+7.56-34.6log10(h_eff)+2.7log10(f)

% NLOS   PL = (44.9 - 6.55log10(h))log10(d)+
%            + 5.83log10(h)+ 18.38 + 23log10 (f)

%% Parameters
h = 1.5;                            % Antenna height [m]
h_eff = h - 1;                      % Effective antenna height [m]
f = 5.9;                            % Central frequency [GHz]
c = 3e8;                            % Speed of light [m/s]
Dbp = 4*h_eff*h_eff*f*10^9/c;       % Breakpoint distance [m]

%% Compute Path Loss

Nvehicles = length(distance(:,1));         % Number of vehicles
LOS = ones(Nvehicles, Nvehicles);

if (~fileObstaclesMap)
    D_corr = 25;                                % Decorrelation distance
    % Without propagation file
    PL_dB = (distance<=Dbp).*(22.7*log10(distance) + 27.0 + 20.0*log10(f))+...
        (distance>Dbp).*(40*log10(distance)+7.56-34.6*log10(h_eff)+2.7*log10(f));
    PL = 10.^(PL_dB/10);
    X = 0;
    Y = 0;
else
    % With propagation file
    D_corr = 10;                                % Decorrelation distance
    
    % Convert coordinates to grid
    [X,Y] = convertToGrid(Xvehicle,Yvehicle,XminMap,YmaxMap,StepMap);
    
    % Checking whether two vehicles are in LOS or NLOS
    for i = 1:Nvehicles
        if Xvehicle(i)~=Inf
            for j = i+1:Nvehicles
                if Xvehicle(i)~=Inf
                    % Compute if there is a wall between two vehicles
                    anyWalls = computeGrid(X(i),Y(i),X(j),Y(j),StepMap,GridMap,true);
                    LOS(i,j) = 1-(anyWalls>0);
                    LOS(j,i) = LOS(i,j);
                end
            end
        end
    end
    
    % Path loss calculation
    PL_dB = (LOS>0).*((distance<=Dbp).*(22.7*log10(distance) + 27.0 + 20.0*log10(f))+...
        (distance>Dbp).*(40*log10(distance)+7.56-34.6*log10(h_eff)+2.7*log10(f)))+...
        (LOS==0).*((44.9-6.55*log10(h))*log10(distance) + 5.83*log10(h)+18.38+23*log10(f));
    PL = 10.^(PL_dB/10); 
end

%% Compute RX power

if(stdDevShadowLOS_dB~=0)
    % Call function to calculate shadowing
    Shadowing_dB = computeShadowing(Shadowing_dB,LOS,dUpdate,stdDevShadowLOS_dB,stdDevShadowNLOS_dB,D_corr);
    Shadowing = 10.^(Shadowing_dB/10);
    
    % Compute channel gain with shadowing
    CHgain = Shadowing./PL;

else

    % Compute channel gain without shadowing
    CHgain = 1./PL;
    
end

% Compute RXpower
RXpower = min(PtxERP_RB(IDvehicle)*Gr,PtxERP_RB(IDvehicle)*Gr.*CHgain);

end

