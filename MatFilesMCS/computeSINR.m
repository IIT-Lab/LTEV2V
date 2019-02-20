function SINR = computeSINR(IDvehicle,BRid,NbeaconsT,NbeaconsF,neighborsID,RXpower,IBEmatrix,Ksi,PtxERP_RB,PnRB)
% Compute SINR of beacons received from vehicles in the awareness range

% NOTE: Various parameters (and not the entire phyParams structure) are passed 
% in input to improve speed

Nv = length(IDvehicle);

% Initialize SINR matrix
SINR = zeros(Nv,Nv-1);

% Find not assigned BRid
indexNOT = BRid<=0;

% Calculate BRidT = vector of BRid in the time domain
BRidT = mod(BRid-1,NbeaconsT)+1;
BRidT(indexNOT) = -1;

% Calculate BRidF = vector of BRid in the frequency domain
BRidF = ceil(BRid/NbeaconsT);
BRidF(indexNOT) = -1;

for i = 1:Nv
    index = find(neighborsID(i,:));
    if ~isempty(index)
        for j = 1:length(index)
            
            % Find BRT and BRF in use
            BRT = BRidT(neighborsID(i,index(j)));
            BRF = BRidF(neighborsID(i,index(j)));
            
            if BRT>0
                
                % Useful received power
                C = RXpower(i,IDvehicle==neighborsID(i,index(j)));
                
                % Initialize interfering power sums vector
                Isums = zeros(NbeaconsF,1);
                
                % Interference computation
                % Find vehicles transmitting in the same subframe
                intIndex = find(BRidT(IDvehicle)==BRT);
                if ~isempty(intIndex)
                    for k = 1:length(intIndex)
                        if intIndex(k)~=i && IDvehicle(intIndex(k))~=neighborsID(i,index(j))
                            % Find which BRF is used by the interferer
                            BRFInt = BRidF(IDvehicle(intIndex(k)));
                            I = RXpower(i,intIndex(k));
                            % Sum interference in that BRF
                            Isums(BRFInt,1) = Isums(BRFInt,1) + I;
                        end
                    end
                end
                
                % Find total interference using IBE
                Itot = IBEmatrix(BRF,:)*Isums;
                
                % Check if the receiver is transmitting on the same BRT
                if BRidT(IDvehicle(i))==BRT
                    % Self-interference
                    selfI = Ksi*PtxERP_RB(IDvehicle(i));
                else
                    % No self-interference
                    selfI = 0;
                end
                
                % SINR computation
                SINR(i,j) = C / (PnRB + selfI + Itot);
                
            end
        end
    end
end

end