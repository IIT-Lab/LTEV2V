function errorMatrixNoBorder = errorRemoveBorder(IDvehicle,errorMatrix,indexNoBorder)
% Delete errors occurring between vehicles at the borders of the scenario

Nerrors = length(errorMatrix(:,1));
errorMatrixNoBorder = zeros(Nerrors,4);
NerrorsNoBorder = 0;

for i = 1:Nerrors
    % If the receiver belongs to the interval without borders
    % tempIndex = index correspondent to ID
    tempIndex = find(IDvehicle==errorMatrix(i,1));                         
    if(isempty(find(indexNoBorder(:,1)==tempIndex,1))==0)
        NerrorsNoBorder = NerrorsNoBorder + 1;
        errorMatrixNoBorder(NerrorsNoBorder,:) = errorMatrix(i,:);
    end    
end

delIndex = errorMatrixNoBorder(:,1)==0;
errorMatrixNoBorder(delIndex,:) = [];

end