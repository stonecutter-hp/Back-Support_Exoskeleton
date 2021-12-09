function EXOmain()
% The high-level flow: 
%   Initialization system parameters --> Serial Initialization -->
%   Handshake with low-level controller --> Set up Timer --> Timer loop:
%     -> Receieve data from MCU -> Control calcualtion -> Send data to MCU
%   --> Timer loop stop (Auto Stop / Manually Stop) --> Delete Serial Port
%   and Timer Items --> Save Data


global ExoP;
% ExoP = Set_Parameters();         % Set necessary parameters
MainFreq = ExoP.MainFreq;          % Set program frequency
% Before start timer, initialize serial port and handshake process
McuPort = ExoP.McuPort;            % Serial Port number for MCU and PC communication
SerialPorts_Init(McuPort);
% Normal
if(ExoP.stopFlag == 0)
    Timer_Init(MainFreq);              % Initialize timer
end

end


