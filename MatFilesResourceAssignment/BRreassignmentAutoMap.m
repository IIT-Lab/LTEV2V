function [BRid, Nreassign, NreassignNoBorder, Nunlocked, NunlockedNoBorder] = BRreassignmentAutoMap(IDvehicle,errorMatrix,awarenessIDReal,awarenessBRid,BRid,Nbeacons,indexNoBorder)
% [AUTONOMOUS CASE with MAP-RP]
% Function that runs Autonomous BR reassignment
% Vehicles try to find an available BR reading maps coming from neighbors
% Blocked vehicles search if there are unused BRs reading received maps

Nerrors = length(errorMatrix(:,1));                      % Number of errors
BRidTemp = BRid;                         % Initialize temporary BRid vector

% Initialize IDs of vehicles that have already searched for a new BR
alreadyReassigned = zeros(Nerrors,1);
w = 1;                                  % Index of alreadyReassigned vector

% Find blocked vehicles
blockedIndex = find(BRid(IDvehicle)==-1);
blockedID = IDvehicle(blockedIndex);
Nb = length(blockedID);

% Reset number of successfully unlocked vehicles
Nunlocked = 0;
NunlockedNoBorder = 0;

% Reset number of successfully reassigned vehicles
Nreassign = 0;
NreassignNoBorder = 0;

% Reassign BRs to blocked vehicles
if Nb~=0
    p1 = randperm(Nb);                 % Generate random permutation vector
    % Blocked vehicles search for free BRs
    for i = 1:Nb
        z = p1(i);
        ID = blockedID(z,1);
        % Initialize vector of occupied BR (0 free, 1 busy)
        BRo = zeros(Nbeacons);
        % Find indexes of vehicles in the awareness range
        rawID = awarenessIDReal(blockedIndex(z,1),:)>0;
        % Indicate the BRs reported in received maps as busy
        for k = 1:length(rawID)
            % If BRid is assigned and the beacon is correctly received (the
            % RX and TX are not in the error matrix), build own map
            if awarenessBRid(blockedIndex(z,1),k)>0 && isempty(find(errorMatrix(:,1)==ID & errorMatrix(:,2)==awarenessIDReal(blockedIndex(z,1),k),1))
                BRo(awarenessBRid(blockedIndex(z,1),k))=1;
                mapID = awarenessIDReal(blockedIndex(z,1),k);
                mapIndex = find(IDvehicle==mapID);
                mapBRIndex = awarenessBRid(mapIndex,:)>0;
                for h = 1:length(mapBRIndex) 
                    % If BRid is assigned and the beacon is correctly received (the
                    % RX and TX are not in the error matrix), add knowledge
                    % of received maps
                    if awarenessBRid(mapIndex,h)>0 && isempty(find(errorMatrix(:,1)==mapID & errorMatrix(:,2)==awarenessIDReal(mapIndex,h),1))
                        BRo(awarenessBRid(mapIndex,h))=1;
                    end
                end
            end
        end
        p2 = randperm(Nbeacons);   % Generate random permutation vector
        for j = 1:Nbeacons         % Check all possible BRs
            BR = p2(j);
            found = 0;  % Becomes 1 if there is a vehicle using that BR in the received maps
            if BRo(BR)==1
                found = 1;
            end
            if found
                BRidTemp(ID) = -1;  % BR not available (blocked)
            else
                BRidTemp(ID) = BR;  % Assign BR
                Nunlocked = Nunlocked + 1;
                if(isempty(find(indexNoBorder(:,1)==blockedIndex(z,1),1))==0)
                    NunlockedNoBorder = NunlockedNoBorder + 1;
                end
                break
            end
        end      
    end
end

% Reassign BRs to vehicles that have been advised that the BR has too much
% interference (beacon cannot be received properly)
if Nerrors~=0
    p3 = randperm(Nerrors);        % Generate random permutation vector
    % Interfering vehicles search for available BRs
    for i = 1:Nerrors
        z = p3(i);
        ID = errorMatrix(z,2);
        % Find the correspondent index in IDvehicle
        IndexVehicle =  find(IDvehicle==ID);
        % Check if the vehicle has already searched for available BRs
        index = find(alreadyReassigned(:,1)==ID, 1);
        if isempty(index)==1
            % Add ID to already reassigned vehicles
            alreadyReassigned(w,1) = ID;
            w = w+1;
            % Initialize vector of occupied BR (0 free, 1 busy)
            BRo = zeros(Nbeacons);
            % Find indexes of vehicles in the awareness range
            rawID = awarenessIDReal(IndexVehicle,:)>0;
            % Indicate the BRs reported in received maps as busy
            for k = 1:length(rawID)
                if awarenessBRid(IndexVehicle,k)>0 && isempty(find(errorMatrix(:,1)==ID & errorMatrix(:,2)==awarenessIDReal(IndexVehicle,k),1))
                    BRo(awarenessBRid(IndexVehicle,k))=1;
                    mapID = awarenessIDReal(IndexVehicle,k);
                    mapIndex = find(IDvehicle==mapID);
                    mapBRIndex = awarenessBRid(mapIndex,:)>0;
                    for h = 1:length(mapBRIndex)
                        if awarenessBRid(mapIndex,h)>0 && isempty(find(errorMatrix(:,1)==mapID & errorMatrix(:,2)==awarenessIDReal(mapIndex,h),1))
                            BRo(awarenessBRid(mapIndex,h))=1;
                        end
                    end
                end
            end
            p4 = randperm(Nbeacons);   % Generate random permutation vector
            for j = 1:Nbeacons         % Check all possible BRs
                BR = p4(j);
                found = 0;  % Becomes 1 if there is a vehicle using that BR in the received maps
                if BRo(BR)==1
                    found = 1;
                end
                if found
                    BRidTemp(ID) = -1;  % BR not available (blocked)
                else
                    BRidTemp(ID) = BR;  % Assign BR
                    Nreassign = Nreassign + 1;
                    if(isempty(find(indexNoBorder(:,1)==IndexVehicle,1))==0)
                        NreassignNoBorder = NreassignNoBorder + 1;
                    end
                    break
                end
            end
        end
    end
end

% Replace BRid with BRidTemp
BRid = BRidTemp;

end
