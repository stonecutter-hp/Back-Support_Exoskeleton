function ProgStart(Ttimer,~)
global ExoP;

McuSerial = ExoP.config{1,1};
% % Flush input buffer
flushinput(McuSerial);
% Start to record time
tic

end