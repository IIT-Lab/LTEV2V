function printPRRmapToFile(simValues,simParams,outParams)
% Print PRRmap to image file

% Find size of PRRmap
[Nrows, Ncolumns] = size(simValues.GridMap);

% Convert limits to map coordinates (if needed)
if simParams.XminTrace <= simValues.XminMap
    XminTraceMap = 1;
else
    [XminTraceMap,~] = convertToGrid(simParams.XminTrace,0,simValues.XminMap,simValues.YmaxMap,simValues.StepMap);
end

if simParams.YminTrace <= simValues.YminMap
    YminTraceMap = Nrows;
else 
    [~,YminTraceMap] = convertToGrid(0,simParams.YminTrace,simValues.XminMap,simValues.YmaxMap,simValues.StepMap);
end

if simParams.XmaxTrace < 0 || simParams.XmaxTrace >= simValues.XmaxMap
    XmaxTraceMap = Ncolumns;
else
    [XmaxTraceMap,~] = convertToGrid(simParams.XmaxTrace,0,simValues.XminMap,simValues.YmaxMap,simValues.StepMap);
end

if simParams.YmaxTrace < 0 || simParams.YmaxTrace >= simValues.YmaxMap
    YmaxTraceMap = 1;
else
    [~,YmaxTraceMap] = convertToGrid(0,simParams.YmaxTrace,simValues.XminMap,simValues.YmaxMap,simValues.StepMap);
end

% Cut the map files
simValues.GridMap = simValues.GridMap(floor(YmaxTraceMap):ceil(YminTraceMap),floor(XminTraceMap):ceil(XmaxTraceMap));
simValues.neighborsMap = simValues.neighborsMap(floor(YmaxTraceMap):ceil(YminTraceMap),floor(XminTraceMap):ceil(XmaxTraceMap));
simValues.correctlyReceivedMap = simValues.correctlyReceivedMap(floor(YmaxTraceMap):ceil(YminTraceMap),floor(XminTraceMap):ceil(XmaxTraceMap));

A = simValues.neighborsMap==0;
% Set -1 as #neighbors where there are no neighbors
simValues.neighborsMap(A)=-1;

% [Linear scale version]
% Create PRRMap matrix
% Set 1 as #correctly received beacons where there are no neighbors 
%simValues.correctlyReceivedMap(A)=1;
%PRRmap = (simValues.correctlyReceivedMap./simValues.neighborsMap).*simValues.GridMap-1.*(1-simValues.GridMap);

% [Non-linear scale]
% Create PRRMap matrix
PRRvalues = (simValues.correctlyReceivedMap./simValues.neighborsMap);
PRRmap = real(((1-sqrt(1-PRRvalues.^2))).*(1-A)-1.*A);

% Print PRRMap to file
filename = sprintf('%s/PRRmap_%.0f.png',outParams.outputFolder,outParams.simID);
createMap(simValues.GridMap,PRRmap,filename);

end
