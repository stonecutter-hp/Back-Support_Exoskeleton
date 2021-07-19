function ProgStart(Ttimer,~)
global ExoP;

McuSerial = ExoP.config{1,1};
% % Flush input buffer
flush(McuSerial,"input");
% Start to record time
tic

end