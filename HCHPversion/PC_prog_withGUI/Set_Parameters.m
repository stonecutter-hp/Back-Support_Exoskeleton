function P = Set_Parameters(MainFreq,MaxRunTime)
% This function is to initialize all the parameters of high-level
% controller
% INPUT: 
%      MainFreq: Expected high-level controller running frequency
%      MaxRunTime: Expected high-level controller running Time
% Notice to ensure the unit of angle is rad/deg first
%% Main program setting information
P.McuPort = "COM3";              % serial port of MCU&PC communication
P.MainFreq = MainFreq;           % main program running frequency (Hz)
P.MaxRunTime = MaxRunTime;       % expected main program running time (s)
P.MaxBendCycles = 1;             % expected bending cycles
P.d2r = pi/180;                  % deg to rad 
%% Configuration information
% P.config{1,1} for serial port configuration
% P.config{2,1} for timer configuration
P.config = cell(2,1);     
% Program stop Flag: 0-normal auto stop; 1-normal manually stop
%                    2-cannot open serial port; 3-handshake error; 
%                    4-serial communication error; 5-Timer callback error
P.stopFlag = 0;
% Stop button pushed flag: 0: No click, 1:clicked
P.stopButton = 0; 
% Serial port status detection flag: every xx cycles detect once
P.serialCycle = 0;

%% For storage of time information saving in TimerCallback
P.TransTime = [];   % time of every send running loop
P.TimeAll = [];     % time point of every loop

%% For prototype operation configuration selection
% UID strategy selection flag
%   0-Test Mode; 1-Basic UID strategy; 2-Developed UID strategy; 
P.UIDStrategy = 1;  
% RTG strategy selection flag:
%   0-Test Mode; 1-Impedance strategy; 2-Gravity compensation;
%   3-Velocity-based compensation strategy
P.RTGStrategy = 1;  
% Subject selection
P.SubjectNum = 1;
% Friction compensation enable flag: 0-disable, 1-enable
P.fricEnable = 0;   

%% For storage of sensor information recieving from MCU
% RecItem marker which should be the same as MCU config with 1 for item
% info ON and 0 for item info OFF
% The order of item corresponds to each marker follows communication
% protocol of MCU2PC:
%          MxTLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxx\r\n 
P.RecItem = [1,    1,    1,     1,    1,    1,     1,    1,    1,    1,    1];
P.torqueTL = [];    % torque feedback of left torsion spring
P.forceLL = [];     % left cable force feedback
P.angleAL = [];     % angle of left hip
P.torqueTR = [];    % torque feedback of right torsion spring
P.forceLR = [];     % right cable force feedback
P.angleAR = [];     % angle of right hip
P.angleP = [];      % pitch angle of turnk bending motion
P.angleY = [];      % yaw angle of trunk bending motion
P.PWM_L = [];       % left motor PWM command
P.PWM_R = [];       % right motor PWM command
% Can be obtained directly from IMU, can also be calculated on PC or on MCU
P.adotPV = [];      % angular velocity of trunk motion in pitch direction from MCU
% Calculated from sensor feedback
P.tdotTL = [];      % time deriviation of left assistive torque
P.tdotTR = [];      % time deriviation of right assistive torque
P.adotAL = [];      % velocity of left hip
P.adotAR = [];      % velocity of right hip



%% For communication protocol
% Delay feedback refers to the feedback which is not following a recieving
% process from PC to MCU, indicating the info package recieved by PC is not
% real-time but with state delay.
% When enable delay feedback, the control and send loop will run as long as
% PC correctly recieved info package from MCU. Otherwise, the control/send
% loop will only run when PC recieved real-time info package without delay.
% Meanwhile, control/send loop will continue to run if over P.MaxDelay loop
% no real-time info package is recieved by PC 

% Total number of char received from MCU, this number should be coincided
% with the MCU2PC communication protocol (include terminator \r\n)
P.ReceiveDataNum = 72;   
% Total number of char send to MCU, this number should be coincided with
% the PC2MCU communication protocol (exclude terminator \r\n) which is
% TLxxxxTRxxxxMxx\r\n (0x0D,0x0A)
P.SendDataNum = 15;       
P.SwitchFlag = '1';      % mark if new recieving is after recieving newest command from PC
P.DelayNumber = 0;       % number of no-newest feedback cycle to guarantee communication correction
P.MaxDelay = 5;          % maximum allowable number of allowable no-newest feedback cycle 
% Enabling flag of delay info package feedback detection: True - Allow
% delay info package, i.e. do not detect if info package is real-time or
% not; False - Detect if the info package is real-time or not, if it is,
% enable Control and Command Sending, if it is not, wait until real-time
% pacakge is received or delay receiving cycle exceed maximum allowable
% cycles P.MaxDelay
P.DelayEnable = true;       
P.DelayMark = [];        % store the delay mark Mx from MCU

% Ready feedback from low-level for handshake, should keep identical to
% low-level setting
P.ReadyFlag = 'Ready.';
P.NotReadyFlag = 'NotReady';

%% For biomechanical model parameter setting of subject
% Data set refers to the P1C draft
P.Subject = [];
P.g = 9.81;              % gravity acceleration

%% For storage of general processed info for UID and RTG strategy
P.Range = 10;            % The range of angle standard deviation 
P.ConThres = 5;          % The number of continous requirment satisfied items at once
% Definition of different motion state:
% mode(1) - state transition indicator:
%                        0-No transition take place
%                        1-State transition has taken place
% mode(2) - motion state:0-Stop state;      1-Exit state; 
%                        2-Standing;        3-Walking;
%                        4-Lowering;        5-Grasping;
%                        6-Lifting;         7-Holding;
% mode(3) - asymmetric direction:
%                        0-none; 1-left; 2-right; 
%                        3 (0+3)-none + fricCom;
%                        4 (1+3)-left + fricCom;
%                        5 (2+3)-right + fricCom
%-------------- State Transition Flag ---------------
P.NoTrans = 0;
P.StateTrans = 1;
% Assume a bending cycle is complete if Lifting -> Standing is detected
% It is reset to 0 if Exit condition is detected
P.BendCycle = 0;   
%------------------- Motion Type --------------------
P.Stop = 1;
P.Exit = 2;
P.Standing = 3;
P.Walking = 4;
P.Lowering = 5;
P.Grasping = 6;
P.Lifting = 7;
P.Holding = 8;
P.MotionModeDis = ['Stop','Exit','Standing','Walking','Lowering','Grasping','Lifting','Holding'];
%--- Asymmetric Direction & Friction Compen Flag ----
P.None = 0;
P.LeftAsy = 1;
P.RightAsy = 2;
P.fricCompen = 3;
% Here there is a initial state assigned as mode = Exit and side = None
P.MotionMode = [P.NoTrans, P.Exit, P.None+P.fricEnable*P.fricCompen];
% From raw sensor feedback without any processing like abs()
P.HipMeanAngle = [];     % (left hip angle + right hip angle)/2
P.HipDiffAngle = [];     % left hip angle - right hip angle
P.HipStdAngle = [];      % (population) std of HipMeanAngle
P.HipStdDiffAngle = [];  % (population) std of HipDiffAngle  
% The trunk bending angle at the moment of lowering, for RTG strategy
P.T0Value = [];
P.TrunkAngleLeftT0 = 0;   
P.TrunkAngleRightT0 = 0;
P.TrunkAngleT0 = (P.TrunkAngleLeftT0 + P.TrunkAngleRightT0)/2;
% The trunk bending angle at the moment of grasping, for RTG strategy
P.PeakValue = [];
P.TrunkAngleLeftPeak = 0;
P.TrunkAngleRightPeak = 0;
P.TrunkAnglePeak = (P.TrunkAngleLeftPeak + P.TrunkAngleRightPeak)/2;

P.DesiredTorque = []; 

%% Specific controller parameters for UID strategy 
P.UID = [];

%% Specific controller parameters for RTG strategy
P.RTG = [];              
% Lower and upper limitation of the reference torque
P.RTGLowerBound = 0;
P.RTGUpperBound = 30;


%% For fiber passive structure model
% P.par1 = 1;       % just for example, maybe for the torque pattern equation parameter

%% For global definition of handles for GUI
% P.appHandles = [];

end