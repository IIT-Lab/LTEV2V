function [updateTimeMatrix,updateDelayCounter] = countUpdateDelay(IDvehicle,BRid,NbeaconsT,awarenessID,errorMatrix,elapsedTime,updateTimeMatrix,updateDelayCounter,delayResolution)
% Function to compute the update delay between received beacons
% Returns the updated updateTimeMatrix and updateDelayCounter

% Update updateTimeMatrix -> matrix containing the timestamp of the last received beacon
% Row index -> receiving vehicle's ID
% Column index -> transmitting vehicle's ID
Nv = length(IDvehicle);
all = 1:length(BRid);
delayMax = length(updateDelayCounter)*delayResolution;

% Calculate BRidT = vector of BRid in the time domain
BRidT = mod(BRid-1,NbeaconsT)+1;

% Reset timestamp of vehicles that are outside the scenario
allIDOut = setdiff(all,IDvehicle);
updateTimeMatrix(allIDOut,:) = -1;

for i = 1:Nv
    % Vehicles inside the awareness range of vehicle with ID i
    IDIn = awarenessID(i,awarenessID(i,:)>0);
    % ID of vehicles that are outside the awareness range of vehicle i
    IDOut = setdiff(all,IDIn);
    updateTimeMatrix(IDvehicle(i),IDOut)=-1;
    for j = 1:length(IDIn)
        % If the vehicle is not blocked and if there is no error in
        % reception, update the matrix with the timestamp of the received
        % beacons
        if BRid(IDIn(j))>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1))
            % Store previous timestamp
            previousTimeStamp = updateTimeMatrix(IDvehicle(i),IDIn(j));
            % Compute current timestamp
            currentTimeStamp = elapsedTime-(0.1/NbeaconsT)*(NbeaconsT-BRidT(IDIn(j)));
            % If there was a previous timestamp
            if previousTimeStamp>0
                % Compute update delay, considering the subframe used for transmission (s)
                updateDelay = currentTimeStamp-previousTimeStamp;
                % Check if the update delay is larger than the maximum delay value stored in the array
                if updateDelay>=delayMax
                    % Increment last counter
                    updateDelayCounter(end) = updateDelayCounter(end) + 1;
                else
                    % Increment counter corresponding to the current delay
                    updateDelayCounter(ceil(updateDelay/delayResolution)) = ...
                        updateDelayCounter(ceil(updateDelay/delayResolution)) + 1;
                end
            end
            % Update updateTimeMatrix with the current timestamp
            updateTimeMatrix(IDvehicle(i),IDIn(j)) = currentTimeStamp;
        end
    end
end

end
