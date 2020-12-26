clear all
clc
% The high-level flow: Initialization --> Receieve data from MCU
%                      --> Control calcualtion --> Send data to MCU
% The low-level controller should keep sending data to PC
% Use stop(P.config{2,1}) to end the whole program

global P;
disp('Program Starts !');

P = Set_Parameters();              % set necessary parameters
MainFreq = P.MainFreq;             % Set program frequency
Timer_Init(MainFreq);              % Initialize timer



