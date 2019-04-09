function [lastGenTimeMatrix,lastSendTimeMatrix,HistogramMatrix] = generateHisMatrix(IDvehicle,awarenessID,awarenessSINR,awarenessBRid,NbeaconsT,distance,gammaMin,elapsedtime,timeNextPacket,lastGenTimeMatrix,lastSendTimeMatrix,HistogramMatrix,binTime,binDist)
% Detect wrongly decoded beacons and create Error Matrix
% [ID RX, ID TX, BRid, distance]

Nv = length(IDvehicle);                   % Total number of vehicles

% ALEX --> I am changing the if condition comparison with double type can give wrong results
% if lastSendTimeMatrix == -1 %Initialization
if size(lastSendTimeMatrix,1) < Nv %Initialization
    lastSendTimeMatrix = -1*ones(size(awarenessBRid)); 
    lastGenTimeMatrix = -1 * ones(size(awarenessBRid));  % ALEX --> I am creating a new matrix because we need GENERATION time AND send time of last BSM received
    
    % ALEX --> This logic is wrong as written because it makes ages be
    % counted twice in the beginning
%     for i = 1:Nv
%         index = find(awarenessID(i,:));
%         for j = 1:length(index)
%             if awarenessSINR(i,index(j))>gammaMin && awarenessBRid(i,index(j))>0
%                 lastSendTimeMatrix(i,index(j))=timeNextPacket(i);   
%             end
%         end
%     end
end    

% ALEX: Calculate BRidT = vector of BRid in the time domain
BRidT = mod(awarenessBRid-1,NbeaconsT)+1;

for i = 1:Nv %i = send vehicle
    index = find(awarenessID(i,:));
    %find the BRid for each receiving vehicles
    %awarenessID is 200x199: 0 stands for no BRid
    if ~isempty(index)
        for j = 1:length(index) %j = receiving vehicles
            % ALEX --> I am changing the conditions in ALL IFs in this
            % loop, because it does not count correctly as written
            
            % If received beacon SINR is lower than the threshold
%            if awarenessSINR(i,index(j))<gammaMin && awarenessBRid(i,index(j))>0
                %do nothing
                %the LastSendTimeMatrix Maintains its value
%            else
            if awarenessSINR(i,index(j))>gammaMin && awarenessBRid(i,index(j))>0 % CHANGED
                % BSM received without error
                    
%                if awarenessSINR(i  ,index(j))>gammaMin && awarenessBRid(i,index(j))>0 && lastSendTimeMatrix(i,index(j))~=-1
                startPoint = lastSendTimeMatrix(i,index(j));
                % ALEX --> changed
                endPoint = elapsedtime - 0.1 + (0.1/NbeaconsT) * BRidT(i,index(j));
%                    endPoint = (elapsedtime+(awarenessBRid(i,index(j))*0.001));   % WRONG
                if lastGenTimeMatrix(i,index(j))~=-1 % CHANGED
                    dis=distance(i,IDvehicle==awarenessID(i,index(j))); %distance between two cars
                    [HistogramMatrix]=addtoHistogram(startPoint,endPoint,lastGenTimeMatrix(i,index(j)),dis,HistogramMatrix,binTime,binDist);  
                end % ADDED
          
                % ALEX --> CHANGED: lastGenTimeMatrix is updated with the generation
                % time of the last packet received
                % And the if was wrong
%                if (timeNextPacket(i)-elapsedtime) > (awarenessBRid(i,index(j))*0.001)  WRONG
                if (timeNextPacket(i)- (elapsedtime-0.1)) > (awarenessBRid(i,index(j))*(0.1/NbeaconsT))
                    %If generation time of received packet is later than the Assined BR
                    lastGenTimeMatrix(i,index(j)) = timeNextPacket(i) - 0.1;
                else
                    %Successfully Sent: Update with timeNextPacket!
                    lastGenTimeMatrix(i,index(j)) = timeNextPacket(i);
                end
                lastSendTimeMatrix(i,index(j)) = endPoint;
            end    % CHANGED
        end
    end 
end

end

% ALEX --> see that I changed the arguments
function [HistogramMatrix]= addtoHistogram(startPoint,endPoint,lastGenTime,distance,HistogramMatrix,binTime,binDist)
% ALEX --> changed while condition
i = startPoint;
while endPoint - i > ( binTime - 0.001 )
    % ALEX --> changed diff, col and row
    diff = i - lastGenTime; % i is the time (incrementing at 0.01s intervals) when you are counting ages, and diff is the age at time i
%    diff=i-startPoint;    WRONG, see above
    col = floor( diff / binTime ) + 1;
%    col = int64(diff*100)+1;     % WRONG: for example, if diff=0.006, then col=2 (0.01-0.02) which is wrong. The correct should be col=1
    row = floor( distance / binDist ) + 1;
%    row = int64(distance/binSize)+1;   % WRONG: for example, if distance=6 and binsize=10, then row=2 (10-20) which is wrong. The correct should be row=1
    [m,n] = size(HistogramMatrix);
    
    if row>m
        addRow = row - m;
        HistogramMatrix = [HistogramMatrix;zeros(addRow,n)];    %Add Rows
    end
    
    if col>n
        addCol=col - n;
        HistogramMatrix = [HistogramMatrix zeros(m,addCol)];  %Add Cols
    end
    
    HistogramMatrix(row,col)=int64(HistogramMatrix(row,col)+1);    
    % ALEX --> this value should be a parameter
    i = i + binTime;
%    i=i+0.01; %slice into 0.01 THIS VALUE SHOULD BE A PARAMETER
end    
end
