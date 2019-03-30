function [lastSendTimeMatrix,HistogramMatrix] = generateHisMatrix(IDvehicle,awarenessID,binSize,awarenessSINR,awarenessBRid,distance,gammaMin,elapsedtime,timeNextPacket,lastSendTimeMatrix,HistogramMatrix)
% Detect wrongly decoded beacons and create Error Matrix
% [ID RX, ID TX, BRid, distance]

Nv = length(IDvehicle);                   % Total number of vehicles


if lastSendTimeMatrix == -1 %Initialization
    lastSendTimeMatrix = -1*ones(size(awarenessBRid));
    for i = 1:Nv
        index = find(awarenessID(i,:));
        for j = 1:length(index)
            if awarenessSINR(i,index(j))>gammaMin && awarenessBRid(i,index(j))>0
                lastSendTimeMatrix(i,index(j))=timeNextPacket(i);   
            end
        end
    end
end    


for i = 1:Nv %i = send vehicle
    index = find(awarenessID(i,:));
    %find the BRid for each receiving vehicles
    %awarenessID is 200x199: 0 stands for no BRid
    if ~isempty(index)
        for j = 1:length(index) %j = receiving vehicles
            % If received beacon SINR is lower than the threshold
            if awarenessSINR(i,index(j))<gammaMin && awarenessBRid(i,index(j))>0
                %do nothing
                %the LastSendTimeMatrix Maintains its value
            else
                if awarenessSINR(i  ,index(j))>gammaMin && awarenessBRid(i,index(j))>0 && lastSendTimeMatrix(i,index(j))~=-1
                    %received!  
                    startPoint = lastSendTimeMatrix(i,index(j));
                    dis=distance(i,IDvehicle==awarenessID(i,index(j))); %distance between two cars
                    endPoint = (elapsedtime+(awarenessBRid(i,index(j))*0.001));
                    [HistogramMatrix]=addtoHistogram(binSize,startPoint,endPoint,dis,HistogramMatrix);  
                    
                    %nextpacket
                    if (timeNextPacket(i)-elapsedtime) > (awarenessBRid(i,index(j))*0.001)
                        %If send packet is later than the Assined BR
                        lastSendTimeMatrix(i,index(j))=timeNextPacket(i) + 0.1;
                    else
                        %Successfully Sent: Update with timeNextPacket!
                        lastSendTimeMatrix(i,index(j)) = timeNextPacket(i);
                    end
               end    
            end    
        end
    end 
end

end

function [HistogramMatrix]= addtoHistogram(binSize,startPoint,endPoint,distance,HistogramMatrix)
i=startPoint;
while i < endPoint 
    diff=i-startPoint; 
    col = int64(diff*100)+1;     
    row = int64(distance/binSize)+1; 
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
    i=i+0.01; %slice into 0.01
end    
end
