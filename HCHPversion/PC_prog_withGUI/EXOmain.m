function EXOmain()
% The high-level flow: 
%   Initialization system parameters --> Serial Initialization -->
%   Handshake with low-level controller --> Set up Timer --> Timer loop:
%     -> Receieve data from MCU -> Control calcualtion -> Send data to MCU
%   --> Timer loop stop (Auto Stop / Manually Stop) --> Delete Serial Port
%   and Timer Item --> Save Data


global ExoP;
% disp('Program Starts !');

% ExoP = Set_Parameters();              % Set necessary parameters
MainFreq = ExoP.MainFreq;             % Set program frequency
Timer_Init(MainFreq);              % Initialize timer
end


