function distanceDetailsCounter = countDistanceDetails(neighborsDistance,neighborsBRid,errorMatrix,distanceDetailsCounter,indexNoBorder)
% Count events for distances up to the maximum awareness range (removing border effect)
% [distance, #Correctly decoded beacons, #Errors, #Blocked neighbors, #Neighbors]
% #Neighbors will be calculated in "printDistanceDetailsCounter.m" (ony one call)

% Remove vehicles at the borders
neighborsDistanceNoBorder = neighborsDistance(indexNoBorder,:);
neighborsBRidNoBorder = neighborsBRid(indexNoBorder,:);

% Update array with the events vs. distance
for i = 1:length(distanceDetailsCounter(:,1))
    % #Assigned BRs
    A = neighborsBRidNoBorder(neighborsDistanceNoBorder<i);
    Nassigned = nnz(A>0);
    % #Errors within i meters
    Nerrors = nnz(errorMatrix(:,4)<i);
    distanceDetailsCounter(i,3) = distanceDetailsCounter(i,3) + Nerrors;
    % #Blocked neighbors within i meters
    Nblocked = nnz(A<0);
    distanceDetailsCounter(i,4) = distanceDetailsCounter(i,4) + Nblocked;
    % #Correctly decoded beacons within i meters
    Ndecoded = Nassigned - Nerrors;
    distanceDetailsCounter(i,2) = distanceDetailsCounter(i,2) + Ndecoded;
end

end
