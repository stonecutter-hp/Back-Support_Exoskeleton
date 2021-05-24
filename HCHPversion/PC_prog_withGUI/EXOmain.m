function EXOmain()
% The high-level flow: 
%   Initialization system parameters --> Handshake with low-level
%   controller 
%   --> Receieve data from MCU 
%                      --> Control calcualtion --> Send data to MCU
% The low-level controller should keep sending data to PC
% Use stop(P.config{2,1}) to end the whole program

global ExoP;
global TempApp;
% disp('Program Starts !');
ExoP = Set_Parameters();              % Set necessary parameters
MainFreq = ExoP.MainFreq;             % Set program frequency
outPutStatus(TempApp,num2str(ExoP.MainFreq));
outPutStatus(TempApp,num2str(ExoP.MaxRunTime));
% Timer_Init(MainFreq);              % Initialize timer
end


