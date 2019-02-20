function [BRid,PtxRB,lambda,Nreassign,NreassignNoBorder,Nunlocked,NunlockedNoBorder] = BRreassignmentControlledPC(IDvehicle,BRid,PtxRB,CHgain,awarenessID,indexNoBorder,Nbeacons,lambda,gammaMin,PnRB,blockTarget,maxPtxRB)
% [CONTROLLED CASE WITH POWER CONTROL]
% Assuming that the network perfectly estimates the channel gain of every
% link, the algorithm tries to maximize the SINR by also using power control

% Number of vehicles
Nv = length(IDvehicle);

% Initialize G matrix
G = zeros(Nv,Nv);

% Build G matrix
for i = 1:Nv
    % Find ID of neighbors of vehicle i
    neighbors = (awarenessID(i,awarenessID(i,:)>0))';
    
    % Find indices of neighbors of vehicle i in IDvehicle
    neighborsIndex = ismember(IDvehicle,neighbors);
    
    for j = 1:Nv
        if j~=i
            % Compute maximum channel gain among the neighbors of i with j as a
            % transmitter (also avoid neighbors with same allocation)
            G(j,i) = max([neighborsIndex(j)*1000 max(CHgain(neighborsIndex,j))]);
        else
            % Compute minimum channel gain among the neighbors of i with i as a
            % transmitter (if there are no neighbors, set a very high value)
            G(j,i) = min([1000 min(CHgain(neighborsIndex,i))]);
        end
    end
end

%% Reassignment Algorithm

% Compute blocking rate
blockRate = 1;

% Convert lambda to dB
lambda_dB = 10*log10(lambda);

% Initialize iteration count
iteration = 1;

% Check if the blocking rate is inside the hysteresys margin: the goal is
% to keep the blocking rate within blockTarget and blockTarget/2

% Thresholds
maxBlock =  blockTarget;
minBlock = blockTarget/2;

% Boolean to indicate if the blocking rate has been higher than the target
blockHigh = false;

% Check conditions on blocking rate and limit to 20 iterations
while ((blockRate>maxBlock) || (blockRate<minBlock && lambda<=gammaMin)) && iteration<=20
    %% Reassignment algorithm
    
    % Increase iteration count by 1
    iteration = iteration + 1;
    
    % Initialize X matrix
    X=zeros(Nv,Nbeacons);
    
    % Create Mutual interference matrix B
    B=zeros(Nv,Nv);
    for i=1:Nv
        for j=1:Nv
            if i ~=j
                B(i,j)=G(j,i)./G(i,i);
            end
        end
    end
    
    % Create Mstar vector
    Mstar = 1:Nv;
    
    % Start algorithm
    for n = 1:Nbeacons
        while ~isempty(Mstar) % While Mstar isempty
            Bstar=B(Mstar,Mstar); % B as a function of Mstar
            sum_vector=sum(Bstar,1);
            [~,idx_j]=min(sum_vector);
            X(Mstar(idx_j),n)=1;
            Mstar(idx_j)=[]; % Delete the element of allocated user
            while ~isempty(Mstar) % While Mstar is empty
                R = zeros(length(Mstar),1); % Initialize R
                i = 1; % Initialize index
                for mm = Mstar
                    X(mm,n) = 1;
                    % Compute H matrix
                    H = lambda * (X*X').*B;
                    raw_sum = sum(H,2);
                    max_raw = max(raw_sum);
                    R(i)=max_raw;
                    X(mm,n)=0;
                    i = i+1;
                end
                [~,idx_i]=min(R);
                X(Mstar(idx_i),n)=1;
                % Recalculate H
                H = lambda * (X*X').*B;
                if max(sum(H,2)) < 1
                    stopexternalwhile = false;
                    Mstar(idx_i) = [] ;
                else
                    X(Mstar(idx_i),n) = 0;
                    stopexternalwhile = true;
                    break
                end
            end
            if stopexternalwhile
                break
            end
        end
        if isempty(Mstar)
            break
        end
    end
    
    % Control if more than one resource is allocated to a vehicle
    if max(sum(X,2))>1
        error('Error: a vehicle has more than one resource allocated');
    end
    
    % From X to resources allocated
    [~,R]=max(X,[],2);
    BRid(IDvehicle) = R.*sum(X,2);
    BRid(BRid==0)=-1;
    
    % Compute again the blocking rate
    blockRate = (nnz(BRid==-1))/Nv;
    
    % Check blocking rate and modify lambda
    if blockRate>maxBlock
        % Set blockHigh to true
        blockHigh = true;
        % Decrease lamdba by 0.5dB to decrease blocking
        lambda_dB = lambda_dB - 0.5;
        % Convert again to linear value
        lambda = 10^(lambda_dB/10);
    elseif blockRate<minBlock
        % Check if the blocking rate has been higher than the target
        if blockHigh==true
            break
        end
        % Increase lamdba by 0.5dB to increase blocking
        lambda_dB = lambda_dB + 0.5;
        % Convert again to linear value
        lambda = 10^(lambda_dB/10);
        if lambda>gammaMin
            lambda = gammaMin;
            break
        end
    end
    
end

% Compute q, H and PtxRB
q = (lambda*PnRB)./diag(G);
H = lambda * (X*X').*B;
p = ((eye(Nv)-H)^-1)*q;

% Normalize power per each resource to maximum allowed
for i = 1:Nbeacons
    index = BRid(IDvehicle)==i;
    p(index) = (p(index)/max(p(index)))*maxPtxRB;
end

% Copy Tx power in PtxRB vector
PtxRB(IDvehicle) = p;

Nreassign = Nv;
NreassignNoBorder = length(indexNoBorder);
Nunlocked = 0;
NunlockedNoBorder = 0;

end