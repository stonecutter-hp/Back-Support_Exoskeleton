function P = Set_Parameters()
% Notice to ensure the unit of angle is rad/deg first
%% Main program setting information
P.McuPort = 'COM4';     % serial port of MCU&PC communication
P.MainFreq = 100;       % main program running frequency (Hz)
P.MaxRunTime = 5;       % main program running time (s)
P.d2r = pi/180;         % deg to rad 
%% Configuration information
% P.config{1,1} for serial port configuration
% P.config{2,1} for timer configuration
P.config = cell(2,1);     

%% For time information saving in TimerCallback
P.TransTime = [];   % time of every send running loop
P.TimeAll = [];     % time point of every loop

%% For sensor information direct from MCU
P.RecItem = [1, 1, 1, 1, 1, 1, 1, 1, 1];
P.torqueTL = [];    % torque feedback of left torsion spring
P.forceLL = [];     % left cable force feedback
P.angleAL = [];     % angle of left thigh
P.torqueTR = [];    % torque feedback of right torsion spring
P.forceLR = [];     % right cable force feedback
P.angleAR = [];     % angle of right thigh
P.angleP = [];      % pitch angle of turnk bending motion
P.angleY = [];      % yaw angle of trunk bending motion
% Can be obtained directly from IMU, can also be calculated on PC or on MCU
P.adotPV = [];      % angular velocity of trunk motion in pitch direction from MCU

%% For high-level strategy of motion phase detection and desired torque generation information saving
P.MotionMode = [];
P.DesiredTorque = [];
P.AlphaMean = [];
P.AlphaDot = [];
P.BetaMean = [];

%% For high-level referenced information configuration
% for motion detection
P.Range = 10;         % Can be the range of angle standard deviation 
P.Alpha_Thre = 0;     % the threshold value of flexion angle
P.AlphaDot_Thre = 0;  % the threshold value of flexion velocity
P.Beta_Thre = 0;      % the threshold value of twisting angle   
P.BetaDot_Thre = 0;   % the threshold value of twisting velocity 

% for torque generation based on biomechanical model
% consider under unit of rad and rad/s and for both side transmission systems
P.GravityKg = 0.3;    % Gravity compensation level coefficient
P.DynamicKd = 20;     % Adjustable dynamic compensation level coeffiecint, here fixed
P.DynamicK = 10;      % Adjustable auxiliary coefficient for dynamic compensation, here fixed 
P.DynamicVmax = 100;  % Adjusted assistive timing coefficient, here fixed

% for torque generation based on impedance regulation
% consider under unit of rad and rad/s and for both side transmission systems
P.ImpedanceKp = 0.5/P.d2r;  % Nm/deg/d2r -> Nm/rad   Virtual stiffness
P.ImpedanceKv = 0/P.d2r;    % Nm*s/deg/d2r -> Nm*s/rad Virtual damping
P.VirAlpha0 = 0*P.d2r;      % deg*d2r -> rad Virtual alpha0
% P.VirAlpha0 = P.Alpha_Thre*P.d2r;
P.VirAlphadot0 = 0*P.d2r;   % deg/s*d2r ->rad/s Virtual alpha0 dot
% P.VirAlphadot0 = P.AlphaDot_Thre*P.d2r;

%% For communication protocol
% Delay feedback refers to the feedback which is not following a recieving
% process from PC to MCU
% When enable delay feedback, the control and send loop will run as long as
% PC recieve correct feedback, otherwise, the control/send loop will only
% run when MCU just recieved the last times' PC command and control/send
% loop will run if over P.MaxDelay loop no command is recieved from PC
P.ReceiveDataNum = 60;   % total number of char receive from MCU
P.SendDataNum = 14;      % total number of char send to MCU 
P.SwitchFlag = '1';      % mark if new recieving is after recieving newest command from PC
P.DelayNumber = 0;       % number of no-newest feedback cycle to guarantee communication correction
P.MaxDelay = 5;          % maximum allowable number of allowable no-newest feedback cycle 
P.DelayEnable = 1;       % enable flag of delay feedback detection
P.DelayMark = [];        % store the delay mark Mx from MCU
%% For biomechanical model parameter setting
% refer to the P1 draft   2020-0331
P.TrunkM = 41;           % unit: kg
P.ArmM = 15;             % unit: kg
P.TrunkHalfL = 0.295;    % unit: m
P.ShoulderHalfL = 0.155; % unit: m
P.ArmL = 0.24;           % unit: m
P.Df = 0.05;             % moment arm for flexion spinal force
P.Dt = 0.05;             % moment arm for twisting spinal force
P.g = 9.81;              % gravity acceleration
P.con = 1.02;            % co-constration index
 

%% For fiber passive structure model
% P.par1 = 1;       % just for example, maybe for the torque pattern equation parameter


end