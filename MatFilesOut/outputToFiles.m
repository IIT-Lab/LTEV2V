function outputToFiles(simVersion,simParams,appParams,phyParams,outParams,outputValues)

outputFolder = outParams.outputFolder;
if isfolder(outputFolder) == false
    mkdir(outputFolder)
end

fileNameMain = sprintf('%s/%s',outputFolder,outParams.outMainFile);
fileMainID = fopen(fileNameMain,'at');

if fseek(fileMainID, 1, 'bof') == -1
    %1 Main settings
    fprintf(fileMainID,'SimID\tSim version\tWhen\tSeed\tSimulated duration\tComputation duration\t');
    fprintf(fileMainID,'File Cfg\t');
    %2 Scenario
    fprintf(fileMainID,'Vehicles position\tFile obstacles map\t');
    fprintf(fileMainID,'Sim (positionTimeResolution,Limits)\t');
    fprintf(fileMainID,'Sim (Mborder,PosError,Tupdate,neighborsSelection,Mvicinity)\t');
    %3 App settings
    fprintf(fileMainID,'App (Tbeacon,Bsize,%%ofRes)\t');
    fprintf(fileMainID,'App (Available BRs,RBsBeacon,sizeSubchannel,AdjacentSCI)\t');
    %4 Phy settings
    fprintf(fileMainID,'Phy (BW,MCS/Mode,Duplex)\t');
    fprintf(fileMainID,'Phy (Ptx,PnRB/PnBW,Gt,Gr,L0,beta,gammaMin_dB)\t');
    fprintf(fileMainID,'Phy (MaxRange2Sigma,RawMaxLOS,RawMaxNLOS,StdDevShadowLOS,StdDevShadowNLOS)\t');
    %5 Algorithm
    fprintf(fileMainID,'Algorithm/Technology\t');
    fprintf(fileMainID,'Alg. params\t');
    %6 Particularly relevant settings
    fprintf(fileMainID,'Raw\tRreuse\tRsense\t');
    %7 Outputs
    fprintf(fileMainID,'Average neighbors\tVariance of neighbors\tAverage vehicles in the scenario\t');
    fprintf(fileMainID,'Average reassignments per vehicle per cycle\tBlocking rate\tError rate\tPacket reception ratio\n');
end

%1 Main settings
fprintf(fileMainID,'%.0f\t%s\t%s\t%.0f\t%f\t%f\t',outParams.simID,simVersion,datestr(now),simParams.seed,simParams.simulationTime,outputValues.computationTime);
fprintf(fileMainID,'%s\t',simParams.fileCfg);

%2 Scenario
if simParams.fileTrace == true
    fprintf(fileMainID,'TraceFile: %s\t',simParams.filenameTrace);
else
    fprintf(fileMainID,'roadLength=%.0f,roadWidth=%.1f,NLanes=%.0f,rho=%.0f,vMean=%.2f,vStDev=%.2f\t',simParams.roadLength,simParams.roadWidth,simParams.NLanes,simParams.rho,simParams.vMean,simParams.vStDev);
end

if simParams.fileObstaclesMap == true
    fprintf(fileMainID,'%s\t',simParams.filenameObstaclesMap);
else
    fprintf(fileMainID,'-\t');
end


if simParams.fileTrace == true
    fprintf(fileMainID,'%f',simParams.positionTimeResolution);
    if simParams.XminTrace~=-1 && simParams.XmaxTrace~=-1
        fprintf(fileMainID,',%.1f<X<%.1f',simParams.XminTrace,simParams.XmaxTrace);
    elseif simParams.XminTrace==-1 && simParams.XmaxTrace>=0
        fprintf(fileMainID,',X<%.1f',simParams.XmaxTrace);
    elseif simParams.XmaxTrace==-1 && simParams.XminTrace>=0
        fprintf(fileMainID,',X>%.1f',simParams.XminTrace);
    end
    if simParams.YminTrace~=-1 && simParams.YmaxTrace~=-1
        fprintf(fileMainID,',%.1f<Y<%.1f',simParams.YminTrace,simParams.YmaxTrace);
    elseif simParams.YminTrace==-1 && simParams.YmaxTrace>=0
        fprintf(fileMainID,',Y<%.1f',simParams.YmaxTrace);
    elseif simParams.YmaxTrace==-1 && simParams.YminTrace>=0
        fprintf(fileMainID,',Y>%.1f',simParams.YminTrace);
    end
    if simParams.XminTrace==-1 && simParams.XmaxTrace==-1 && simParams.YminTrace==-1 && simParams.YmaxTrace==-1
       fprintf(fileMainID,',-'); 
    end
else
    fprintf(fileMainID,'%f,-',simParams.positionTimeResolution);
end

fprintf(fileMainID,'\t');

fprintf(fileMainID,'%.0f,',simParams.Mborder);
if simParams.useLTE
    fprintf(fileMainID,'%.1f,',simParams.posError95);
    if simParams.Tupdate > simParams.simulationTime
        fprintf(fileMainID,'inf,');
    else
        fprintf(fileMainID,'%f,',simParams.Tupdate);
    end
else
    fprintf(fileMainID,'-,-,');
end

if simParams.neighborsSelection
    fprintf(fileMainID,'true,%0.f',simParams.Mvicinity);
else
    fprintf(fileMainID,'false,-');
end

fprintf(fileMainID,'\t');

%3 App settings
if simParams.useLTE
    fprintf(fileMainID,'%.3f,%.0f,%.0f\t',appParams.Tbeacon,appParams.beaconSizeBytes,appParams.resourcesV2V);
    fprintf(fileMainID,'%.0f,%.0f,%.0f,',appParams.Nbeacons,appParams.RBsBeacon,phyParams.sizeSubchannel);
    if phyParams.ifAdjacent == true
        fprintf(fileMainID,'true\t');
    else
        fprintf(fileMainID,'false\t');
    end
else
    fprintf(fileMainID,'%.3f,%.0f,-\t',appParams.Tbeacon,appParams.beaconSizeBytes);
    fprintf(fileMainID,'-,-,-,-\t');
end

%4 Phy settings
if simParams.useLTE
    fprintf(fileMainID,'%.1f,%.0f,%s',phyParams.BwMHz,phyParams.MCS,phyParams.duplex);
    if strcmp(phyParams.duplex,'FD')
        fprintf(fileMainID,' (Ksi=%.0fdB)',phyParams.Ksi_dB);
    end
else
    fprintf(fileMainID,'%.1f,%.0f,-',phyParams.BwMHz,phyParams.Mode);
end
fprintf(fileMainID,'\t');

fprintf(fileMainID,'%.0f,',phyParams.Ptx_dBm);
if simParams.useLTE
    fprintf(fileMainID,'%.0f,',phyParams.PnRB_dBm);
else
    fprintf(fileMainID,'%.0f,',phyParams.PnBW_dBm);
end
fprintf(fileMainID,'%.0f,%.0f,',phyParams.Gt_dB,phyParams.Gr_dB);

if ~phyParams.winnerModel
    fprintf(fileMainID,'%.0f,%.3f,%.2f',phyParams.L0_dB,phyParams.beta,phyParams.gammaMin_dB);
    if simParams.fileObstaclesMap == true
        fprintf(fileMainID,' (Abuild=%.2fdB,Awall=%.2fdB)',phyParams.Abuild_dB,phyParams.Awall_dB);
    end
else
    fprintf(fileMainID,'-,-,%.2f (Winner+)',phyParams.gammaMin_dB);
end
fprintf(fileMainID,'\t');

if phyParams.winnerModel
    fprintf(fileMainID,'%.0f,%.0f,%.0f,%.0f,%.0f\t',phyParams.RawMax,phyParams.RawMaxLOS,phyParams.RawMaxNLOS,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
else
    fprintf(fileMainID,'%.0f,%.0f,-,%.0f,%.0f\t',phyParams.RawMax,phyParams.RawMaxLOS,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
end

%5 Algorithm
if simParams.useLTE
    fprintf(fileMainID,'%d\t',simParams.BRAlgorithm);
    if simParams.BRAlgorithm==1 || simParams.BRAlgorithm==2
        if simParams.randomOrder==true
            fprintf(fileMainID,'rand');
        else
            fprintf(fileMainID,'order');
        end
        fprintf(fileMainID,',margin %.0f',simParams.Mreuse);
    end
    if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==7 || simParams.BRAlgorithm==9
        fprintf(fileMainID,',Treassign=%.1f',simParams.Treassign);
    elseif simParams.BRAlgorithm==5
        fprintf(fileMainID,'p=%.2f,k=%d,M=%ddB',simParams.pReselect,simParams.kBest,simParams.hysteresysM_dB);
    elseif simParams.BRAlgorithm==6
        fprintf(fileMainID,'Tsps=%.2f,M=%d',simParams.Tsps,simParams.MBest);
    elseif simParams.BRAlgorithm==8
        fprintf(fileMainID,'NsensPer=%d,pKeep=%.2f,',simParams.NsensingPeriod,simParams.probResKeep);
        fprintf(fileMainID,'rRes=%.2f,minR=%d,maxR=%d,',simParams.ratioSelectedMode4,simParams.minRandValueMode4,simParams.maxRandValueMode4);
        fprintf(fileMainID,'T1=%d,T2=%d,',simParams.subframeT1Mode4,simParams.subframeT2Mode4);
        fprintf(fileMainID,'Pthr=%d,minSCIsinr=%.2f',10*log10(simParams.powerThresholdMode4)+30,10*log10(phyParams.minSCIsinr));
    end
    fprintf(fileMainID,'\t');
else
    fprintf(fileMainID,'802.11p\t');
    fprintf(fileMainID,'-\t');
end

%6 Particularly relevant settings
fprintf(fileMainID,'%.0f\t',phyParams.Raw);

if simParams.useLTE && (simParams.BRAlgorithm==1 || simParams.BRAlgorithm==2)
    fprintf(fileMainID,'%.0f\t',phyParams.Rreuse);
else
    fprintf(fileMainID,'-\t');
end

if simParams.useLTE && simParams.BRAlgorithm==3
    fprintf(fileMainID,'%.0f\t',phyParams.Rsense);
else
    fprintf(fileMainID,'-\t');
end

%7 Outputs
fprintf(fileMainID,'%f\t%f\t%f\t%0.f\t',outputValues.NneighborsNoBorderTOT,outputValues.StDevNeighboursNoBorderTOT,outputValues.AvgNvehiclesNoBorder);
fprintf(fileMainID,'%f\t%f\t%f\t%f\n',outputValues.NreassignNoBorderTOT,outputValues.blockingRateNoBorderTOT,outputValues.errorRateNoBorderTOT,outputValues.packetReceptionRatioTOT);

