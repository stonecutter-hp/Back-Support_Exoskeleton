function ProgStart(Ttimer,~)
global P;

% Initialize Serial Port
McuPort = P.McuPort;          % Serial Port number for MCU and PC communication
SerialPorts_Init(McuPort);
McuSerial = P.config{1,1};
% Flush input buffer
flushinput(McuSerial);
% Start to record time
tic

end