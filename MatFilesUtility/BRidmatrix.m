function awarenessBRid = BRidmatrix(awarenessID,BRid)
% Create awareness BRid matrix

Nvehicles = length(awarenessID(:,1));   % Total number of vehicles

awarenessBRid = zeros(Nvehicles,Nvehicles-1);
for i = 1:Nvehicles
    j=1;
    while(j<=Nvehicles-1 && awarenessID(i,j))
        awarenessBRid(i,j) = BRid(awarenessID(i,j),1);
        j=j+1;
    end
end

end