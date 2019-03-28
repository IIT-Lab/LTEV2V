<<<<<<< HEAD
function [lastSendTimeMatrix,HistogramMatrix] = generateHisMatrix(IDvehicle,awarenessID,binSize,awarenessSINR,awarenessBRid,distance,gammaMin,elapsedtime,timeNextPacket,lastSendTimeMatrix,HistogramMatrix)
=======
function [lastSendTimeMatrix,firstPacketTransmitted,HistogramMatrix] = generateHisMatrix(IDvehicle,awarenessID,binSize,awarenessSINR,awarenessBRid,distance,gammaMin,elapsedtime,timeNextPacket,lastSendTimeMatrix,firstPacketTransmitted,HistogramMatrix)
>>>>>>> 771daf6029ada29759c31ea0c9c6174f8a6ef75e
% Detect wrongly decoded beacons and create Error Matrix
% [ID RX, ID TX, BRid, distance]

Nv = length(IDvehicle);                   % Total number of vehicles

<<<<<<< HEAD

if lastSendTimeMatrix == 0 %Initialization
    lastSendTimeMatrix = zeros(size(awarenessBRid));
=======
ageMatrix = zeros(size(awarenessBRid));     % ALEX: you are not using this matrix to calc age for the histogram, right? What is it for?

if firstPacketTransmitted == 0
    firstPacketTransmitted = zeros(size(awarenessBRid));
end    
    

if lastSendTimeMatrix == 0
    lastSendTimeMatrix = awarenessBRid;
>>>>>>> 771daf6029ada29759c31ea0c9c6174f8a6ef75e
    for i = 1:Nv
        index = find(awarenessID(i,:));
        for j = 1:length(index)
            lastSendTimeMatrix(i,index(j))=timeNextPacket(i);   
        end
    end
end    

<<<<<<< HEAD



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
                if awarenessSINR(i  ,index(j))>gammaMin && awarenessBRid(i,index(j))>0
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
=======
for i = 1:Nv
    index = find(awarenessID(i,:));
    if ~isempty(index)
        for j = 1:length(index)
            
            % If received beacon SINR is lower than the threshold
            if awarenessSINR(i,index(j))<gammaMin && awarenessBRid(i,index(j))>0
                %do nothing
            else
                if awarenessSINR(i  ,index(j))>gammaMin && awarenessBRid(i,index(j))>0
                    %received!
                    ageMatrix(i,index(j)) = (elapsedtime+(awarenessBRid(i,index(j))*0.001)) - (lastSendTimeMatrix(i,index(j)));
                    startPoint = lastSendTimeMatrix(i,index(j));
                    dis=distance(i,IDvehicle==awarenessID(i,index(j)));    
                    endPoint = (elapsedtime+(awarenessBRid(i,index(j))*0.001));
                    [HistogramMatrix]=addtoHistogram(binSize,startPoint,endPoint,dis,HistogramMatrix);  
                    %nextpacket
                    %lastSendTimeMatrix(i,index(j)) = timeNextPacket(i);   % ALEX: are there cases where timeNextPacket > BRid? If so, then lastSendTimeMatrix(i,index(j))=timeNextPacket(i) + 0.1
                    if timeNextPacket(i)-elapsedtime<awarenessBRid(i,index(j))*0.001
                        lastSendTimeMatrix(i,index(j))=timeNextPacket(i) + 0.1;
                    else
>>>>>>> 771daf6029ada29759c31ea0c9c6174f8a6ef75e
                        lastSendTimeMatrix(i,index(j)) = timeNextPacket(i);
                    end
               end    
            end    
        end
    end 
end

<<<<<<< HEAD
=======


%%Process the ageMatrix for each simulation cycle
%Get the non zero elements (Succeesful Age)
% ageMatrix= nonzeros(ageMatrix);
% 
% res = [res ; ageMatrix];


>>>>>>> 771daf6029ada29759c31ea0c9c6174f8a6ef75e
end

function [HistogramMatrix]= addtoHistogram(binSize,startPoint,endPoint,distance,HistogramMatrix)
i=startPoint;
<<<<<<< HEAD
while i < endPoint 
    diff=i-startPoint; 
    col = int64(diff*100)+1;     
    row = int64(distance/binSize)+1; 
    [m,n] = size(HistogramMatrix);
    if row>m
        addRow = row - m;
        HistogramMatrix = [HistogramMatrix;zeros(addRow,n)];    %Add Rows
=======
while i < endPoint
    diff=i-startPoint; 
    col = int64(diff*100)+1;     
    row = int64(distance/binSize)+1;  % same as above. And please use a smaller bin, e.g. 10 m
    [m,n] = size(HistogramMatrix);
    if row>m
        addRow = row - m;
        HistogramMatrix = [HistogramMatrix;zeros(addRow,n)];    % ALEX: this is ok but expanding a matrix every step makes the code slower. We already know that the number of rows is (awareness range) / (bin size), so it is faster to initialize the matrix with all rows
>>>>>>> 771daf6029ada29759c31ea0c9c6174f8a6ef75e
    end
    
    if col>n
        addCol=col - n;
<<<<<<< HEAD
        HistogramMatrix = [HistogramMatrix zeros(m,addCol)];  %Add Cols
    end
    
    HistogramMatrix(row,col)=int64(HistogramMatrix(row,col)+1);    
    i=i+0.01; %slice into 0.01
=======
        HistogramMatrix = [HistogramMatrix zeros(m,addCol)]; 
    end
    HistogramMatrix(row,col)=int64(HistogramMatrix(row,col)+1);    
    i=i+0.01;
>>>>>>> 771daf6029ada29759c31ea0c9c6174f8a6ef75e
end    
end