function timeNextTxRx = resumeBackoff11p(timeNow,nSlotBackoff,tAifs,tSlot)

timeNextTxRx = timeNow + tAifs + nSlotBackoff * tSlot;

end