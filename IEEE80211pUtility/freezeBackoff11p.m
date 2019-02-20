function nSlotBackoff = freezeBackoff11p(timeNow,timeNextTxRx,tSlot,nSlotBackoff)

nSlotBackoff = max( min(ceil((timeNextTxRx-timeNow)/tSlot),nSlotBackoff) ,0);

end