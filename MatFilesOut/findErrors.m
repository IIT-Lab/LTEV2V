function [errorMatrix,resultsID,res,lastSendTimeMatrix,firstPacketTransmitted,HistogramMartix] = findErrors(IDvehicle,awarenessID,awarenessSINR,awarenessBRid,distance,gammaMin,elapsedtime,timeNextPacket,lastSendTimeMatrix,res,firstPacketTransmitted,HistogramMartix)
% Detect wrongly decoded beacons and create Error Matrix
% [ID RX, ID TX, BRid, distance]

Nv = length(IDvehicle);                   % Total number of vehicles
errorMatrix = zeros(Nv*Nv-1,4);           % Initialize collision matrix
Nerrors = 0;                              % Initialize number of errors


ageMatrix = zeros(size(awarenessBRid));

if firstPacketTransmitted == 0
    firstPacketTransmitted = zeros(size(awarenessBRid));
end    
    
if lastSendTimeMatrix == 0
    lastSendTimeMatrix = awarenessBRid;
    for i = 1:Nv
        index = find(awarenessID(i,:));
        for j = 1:length(index)
            lastSendTimeMatrix(i,index(j))=timeNextPacket(i);
        end
    end
end    

resultsID = awarenessBRid;
wrongVehicle = [0];
for i = 1:Nv
    index = find(awarenessID(i,:));
    if ~isempty(index)
        for j = 1:length(index)
            dis=distance(i,IDvehicle==awarenessID(i,index(j)));

            % If received beacon SINR is lower than the threshold
            if awarenessSINR(i,index(j))<gammaMin && awarenessBRid(i,index(j))>0
                Nerrors = Nerrors + 1;
                errorMatrix(Nerrors,1) = IDvehicle(i);
                errorMatrix(Nerrors,2) = awarenessID(i,index(j));
                errorMatrix(Nerrors,3) = awarenessBRid(i,index(j));
                errorMatrix(Nerrors,4) = distance(i,IDvehicle==awarenessID(i,index(j)));
                wrongVehicle = [wrongVehicle,IDvehicle(i)];
                resultsID(i,index(j)) = -1; %failed communication no BRid
            else
                if awarenessSINR(i,index(j))>gammaMin && awarenessBRid(i,index(j))>0 && dis<=150
                    resultsID(i,index(j)) = awarenessBRid(i,index(j));
                    %received!
                    ageMatrix(i,index(j)) = (elapsedtime+(awarenessBRid(i,index(j))*0.001)) - (lastSendTimeMatrix(i,index(j)));
                    startPoint = lastSendTimeMatrix(i,index(j));
                    endPoint = (elapsedtime+(awarenessBRid(i,index(j))*0.001));
                    [HistogramMartix]=addtoHistogram(startPoint,endPoint,dis,HistogramMartix);  
                    %nextpacket
                    if timeNextPacket(i)>elapsedtime+awarenessBRid(i,index(j))*0.001
                        lastSendTimeMatrix(i,index(j)) = timeNextPacket(i)+0.1
                    else    
                        lastSendTimeMatrix(i,index(j)) = timeNextPacket(i);
                    end
                    
                end    
            end    
        end
    end 
end



%%Process the ageMatrix for each simulation cycle
%Get the non zero elements (Succeesful Age)
ageMatrix= nonzeros(ageMatrix);

res = [res ; ageMatrix];

delIndex = errorMatrix(:,1)==0;
errorMatrix(delIndex,:) = [];

end

function [HistogramMartix]= addtoHistogram(startPoint,endPoint,distance,HistogramMartix)
binsize = 10;
i=startPoint;
while i < endPoint
    diff=i-startPoint; 
    col = int16(diff*100)+1;
    row = int16(distance/binsize)+1;
    %distance will neve exceed 150m
    
    HistogramMartix(row,col)=HistogramMartix(row,col)+1;    
    i=i+0.01;
end    
end