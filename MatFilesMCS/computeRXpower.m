function [RXpower,CHgain,Shadowing_dB,X,Y] = computeRXpower(IDvehicle,distance,PtxERP_RB,Gr,L0,beta,Xvehicle,Yvehicle,Abuild,Awall,XminMap,YmaxMap,StepMap,GridMap,fileObstaclesMap,Shadowing_dB,dUpdate,stdDevShadowLOS_dB,stdDevShadowNLOS_dB)
% Compute received power and create RXpower matrix

%% Check if it is present the obstacles map file and calculate supplementary attenuation

Nvehicles = length(distance(:,1));   % Number of vehicles
A = ones (Nvehicles,Nvehicles);

if fileObstaclesMap==true
    
    D_corr = 10;           % Decorrelation distance
    
    % Convert coordinates to grid
    [X,Y] = convertToGrid(Xvehicle,Yvehicle,XminMap,YmaxMap,StepMap);
    
    % Compute attenuation due to walls and buildings
    for i = 1:Nvehicles
        if Xvehicle(i)~=Inf
            for j = i+1:Nvehicles
                if Xvehicle(j)~=Inf
                    [Nwalls,Nsteps,granularity] = computeGrid(X(i),Y(i),X(j),Y(j),StepMap,GridMap,false);
                    A(i,j) = (Awall^Nwalls)*(Abuild^(Nsteps*granularity));
                    A(j,i) = A(i,j);
                end
            end
        end
    end
else
    D_corr = 25;         % Decorrelation distance
    X = 0;
    Y = 0;
end

%% Compute RXpower

if (stdDevShadowLOS_dB~=0)
    
    LOS = A<=1;
    % Call function to calculate shadowing
    Shadowing_dB = computeShadowing(Shadowing_dB,LOS,dUpdate,stdDevShadowLOS_dB,stdDevShadowNLOS_dB,D_corr);
    Shadowing = 10.^(Shadowing_dB/10);
    
    % Compute channel gain with shadowing
    CHgain = Shadowing./(L0*((distance).^beta))./A;
    
else
    
    % Compute channel gain without shadowing
    CHgain = 1/(L0*((distance).^beta))./A;  
    
end

% Compute RXpower
RXpower = min(PtxERP_RB(IDvehicle)*Gr,PtxERP_RB(IDvehicle)*Gr.*CHgain);

end