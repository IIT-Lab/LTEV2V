function printNeighborsToFile(NneighborsRaw,outParams)
% Print number of neighbors per vehicle at each snapshot for traffic trace
% analysis (CDF plot)

filename8 = sprintf('%s/neighbors_%.0f.xls',outParams.outputFolder,outParams.simID);
fileID = fopen(filename8,'at');

for k = 1:length(NneighborsRaw)
    fprintf(fileID,'%d\n',NneighborsRaw(k,1));
end

fclose(fileID);

end

