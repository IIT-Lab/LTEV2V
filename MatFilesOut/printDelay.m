function printDelay(outputValues,outParams)
% Print to file the delay occurrences
% [delay (s) - number of events - CDF]

% Update delay
if outParams.printUpdateDelay
    filename = sprintf('%s/update_delay_%.0f.xls',outParams.outputFolder,outParams.simID);
    fileID = fopen(filename,'at');
    
    NeventsTOT = sum(outputValues.updateDelayCounter);
    
    for i = 1:length(outputValues.updateDelayCounter)
        fprintf(fileID,'%.3f\t%d\t%.6f\n',i*outParams.delayResolution,outputValues.updateDelayCounter(i),sum(outputValues.updateDelayCounter(1:i))/NeventsTOT);
    end
    
    fclose(fileID);
end

% Packet delay
if outParams.printPacketDelay
    filename = sprintf('%s/packet_delay_%.0f.xls',outParams.outputFolder,outParams.simID);
    fileID = fopen(filename,'at');
    
    NeventsTOT = sum(outputValues.packetDelayCounter);
    
    for i = 1:length(outputValues.packetDelayCounter)
        fprintf(fileID,'%.3f\t%d\t%.6f\n',i*outParams.delayResolution,outputValues.packetDelayCounter(i),sum(outputValues.packetDelayCounter(1:i))/NeventsTOT);
    end
    
    fclose(fileID);
end

end

