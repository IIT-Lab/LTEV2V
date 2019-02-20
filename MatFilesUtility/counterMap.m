function  simValues = counterMap(simValues,awarenessMatrix)
% Function to update matrices needed for PRRmap creation in urban scenarios

Nvehicles = length(simValues.XmapFloor);

for i = 1:Nvehicles
    simValues.correctlyReceivedMap(simValues.YmapFloor(i),simValues.XmapFloor(i)) = ...
        simValues.correctlyReceivedMap(simValues.YmapFloor(i),simValues.XmapFloor(i))+awarenessMatrix(i,1);
    simValues.neighborsMap(simValues.YmapFloor(i),simValues.XmapFloor(i)) = ... 
        simValues.neighborsMap(simValues.YmapFloor(i),simValues.XmapFloor(i))+awarenessMatrix(i,4);
end

end