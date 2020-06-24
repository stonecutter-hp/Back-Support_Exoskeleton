function P = Set_Parameters()
%% Main program setting information
P.McuPort = 'COM4';  % serial port of MCU&PC communication
P.MainFreq = 200;   % main program running frequency (Hz)
P.MaxRunTime = 5;   % main program running time

%% Configuration information
% P.config{1,1} for serial port configuration
% P.config{2,1} for timer configuration
P.config = cell(2,1);     

%% For time information saving in TimerCallback
P.TransTime = 0;
P.TimeAll = [];

%% For sensor information saving from MCU
P.angleAL = [];    % angle of left side motor
P.angleSL = [];    % speed of left side motor
P.angleTL = [];    % torque of left side motor
P.angleAR = [];    % angle of right side motor
P.angleSR = [];    % speed of right side motor
P.angleTR = [];    % torque of right side motor
P.angleR = [];    % roll angle
P.angleP = [];    % pitch angle
P.angleY = [];    % yaw angle


%% For high-level strategy of motion phase detection and desired torque generation information saving
P.MotionMode = [];
P.DesiredTorque = [];
P.AngleMean = [];
P.AngleDot = [];
P.YawAngle = [];

%% For high-level referenced information configuration
% for motion detection
P.Range = 10;    % Can be the range of angle standard deviation 
P.Alpha_Thre = 0;   % the threshold value of flexion angle
P.AlphaDot_Thre = 0;  % the threshold value of flexion velocity
P.Beta_Thre = 0;    % the threshold value of twisting angle   
P.BetaDot_Thre = 0;  % the threshold value of twisting velocity 

% for torque generation 
P.GravityKg = 0.3;
P.DynamicKd = 60;
P.DynamicK = 1.5;
P.DynamicMaxV = 1.3;   % rad/s
 
%% For communication protocol
P.ReceiveDataNum = 27;   % total number of char receive from MCU
P.SendDataNum = 14;      % total number of char send to MCU 

%% For biomechanical model parameter setting
% refer to the P1 draft   2020-0331
P.TrunkM = 41;   % unit: kg
P.ArmM = 15;     % unit: kg
P.TrunkHalfL = 0.295;  % unit: m
P.ShoulderHalfL = 0.155;  % unit: m
P.ArmL = 0.24;    % unit: m
P.Df = 0.05;    % moment arm for flexion spinal force
P.Dt = 0.05;    % moment arm for twisting spinal force
P.g = 9.81;     % gravity acceleration
P.con = 1.02;   % co-constration index


%% For fiber passive structure model
% P.par1 = 1;       % just for example, maybe for the torque pattern equation parameter


end