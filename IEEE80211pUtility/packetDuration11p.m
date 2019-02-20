function tPack = packetDuration11p(Nbyte,Mode) 
% IEEE 802.11p: find packet transmission duration

% ns = bits per OFDM symbol 
if Mode==1
    ns = 24; % Mode 1
elseif Mode==2
    ns = 36; % Mode 2
elseif Mode==3
    ns = 48; % Mode 3
elseif Mode==4
    ns = 72; % Mode 4
elseif Mode==5
    ns = 96; % Mode 5
elseif Mode==6
    ns = 144; % Mode 6
elseif Mode==7
    ns = 192; % Mode 7
elseif Mode==8
    ns = 216; % Mode 8
else
    error('Error');
end
Nol = 22; % overhead at MAC in bits
%Noh = 60; % overhead above MAC in bytes
tOs = 8e-6; % OFDM symbol duration
tPreamble = 40e-6; % overhead at PHY in us

tPack = tPreamble + ceil ( (Nbyte*8 + Nol )/ns ) * tOs;
