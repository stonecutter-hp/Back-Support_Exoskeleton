function ProgStart(Ttimer,~)
global P;
McuSerial = P.config{1,1};
% Flush input buffer
flushinput(McuSerial);
% Start to record time
tic

end