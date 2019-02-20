function errorMatrix = findErrors(IDvehicle,awarenessID,awarenessSINR,awarenessBRid,distance,gammaMin)
% Detect wrongly decoded beacons and create Error Matrix
% [ID RX, ID TX, BRid, distance]

Nv = length(IDvehicle);                   % Total number of vehicles
errorMatrix = zeros(Nv*Nv-1,4);           % Initialize collision matrix
Nerrors = 0;                              % Initialize number of errors

for i = 1:Nv
    index = find(awarenessID(i,:));
    if ~isempty(index)
        for j = 1:length(index)
            % If received beacon SINR is lower than the threshold
            if awarenessSINR(i,index(j))<gammaMin && awarenessBRid(i,index(j))>0
                Nerrors = Nerrors + 1;
                errorMatrix(Nerrors,1) = IDvehicle(i);
                errorMatrix(Nerrors,2) = awarenessID(i,index(j));
                errorMatrix(Nerrors,3) = awarenessBRid(i,index(j));
                errorMatrix(Nerrors,4) = distance(i,IDvehicle==awarenessID(i,index(j)));
            end      
        end
    end 
end

delIndex = errorMatrix(:,1)==0;
errorMatrix(delIndex,:) = [];

end
