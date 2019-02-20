function [simParams,simValues] = loadObstaclesMapFile(simParams,simValues)
% Function to load the Ostacles Map File (map of roads and buildings)

tic

fprintf('\nLoading Obstacles Map File\n');

fileID = fopen(simParams.filenameObstaclesMap,'r');

% Read the first line of the file
Info = fscanf(fileID,'%d,%d,%d,%d,\n');

simValues.XminMap = Info(1);
simValues.XmaxMap = Info(2);
simValues.YminMap = Info(3);
simValues.YmaxMap = Info(4);
simValues.StepMap = Info(5);

Ncolumns = (simValues.XmaxMap-simValues.XminMap)/simValues.StepMap;
Nrows = (simValues.YmaxMap-simValues.YminMap)/simValues.StepMap;

% Copy all columns in a char matrix
stringMap = fscanf(fileID,'%s\n',[Ncolumns Nrows]);

% Initialize output matrix
simValues.GridMap = zeros(Nrows,Ncolumns);

% Do the transpose and copy the values to the output matrix
for i = 1:Ncolumns
    for j = 1:Nrows
        simValues.GridMap(j,i) = str2double(stringMap(i,j));
    end
end

toc

end
