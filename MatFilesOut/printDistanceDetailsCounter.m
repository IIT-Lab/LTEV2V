function printDistanceDetailsCounter(distanceDetailsCounter,outParams)
% Print to file Rx details vs. distance up to Raw Max

% #Neighbors within i meters
distanceDetailsCounter(:,5) = distanceDetailsCounter(:,2) + distanceDetailsCounter(:,3) + distanceDetailsCounter(:,4);

filename = sprintf('%s/distance_details_%.0f.xls',outParams.outputFolder,outParams.simID);
fileID = fopen(filename,'at');

for i = 1:length(distanceDetailsCounter(:,1))
    fprintf(fileID,'%d\t%d\t%d\t%d\t%d\n',distanceDetailsCounter(i,1),distanceDetailsCounter(i,2),distanceDetailsCounter(i,3),distanceDetailsCounter(i,4),distanceDetailsCounter(i,5));
end

fclose(fileID);

end
