function matrix = counter(IDvehicle,awarenessBRid,errorMatrix)
% Count number of assigned BRs in the range of each vehicle
% Count number of blocked vehicles in the range of each vehicle
% Matrix = [#Correctly decoded beacons, #Errors, #Blocked neighbors, #Neighbors]

Nvehicles = length(IDvehicle);
matrix = zeros(Nvehicles,4);
for i = 1:Nvehicles
    assigned = nnz(awarenessBRid(i,:)>0);
    errors = nnz(errorMatrix(:,1)==IDvehicle(i));
    blocked = nnz(awarenessBRid(i,:)==-1);
    matrix(i,1) = assigned - errors;
    matrix(i,2) = errors;
    matrix(i,3) = blocked;
    matrix(i,4) = nnz(awarenessBRid(i,:));
end

end