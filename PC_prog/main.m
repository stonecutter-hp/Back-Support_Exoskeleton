clear all
clc
% The high-level flow: Initialization --> Receieve data from MCU
%                      --> Control calcualtion --> Send data to MCU
% The low-level controller should keep sending data to PC
global P;
disp('Program Starts !');

P = Set_Parameters();              % set necessary parameters
McuPort = P.McuPort;               % Serial Port number for MCU and PC communication
SerialPorts_Init(McuPort);       % Initialize Serial Port
MainFreq = P.MainFreq;             % Set program frequency
Timer_Init(MainFreq);            % Initialize timer



