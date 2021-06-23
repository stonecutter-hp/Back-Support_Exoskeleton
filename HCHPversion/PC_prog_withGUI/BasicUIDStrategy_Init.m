function P = BasicUIDStrategy_Init()
% Initialize the specific parameters of the basic UID strategy
%% Thresholds of basic UID strategy 
P.HipMeanAngle_Thre = [0, 0, 0];     % Threshold of hip mean angle
P.HipDiffAngle_Thre = 0;             % Threshold of hip diff angle
P.HipStdAngle_Thre = [0, 0, 0, 0];   % Threshold of std of hip mean angle
P.Alpha_Thre = [0, 0];               % Threshold of flexion angle
P.AlphaDot_Thre = [0, 0, 0];         % Threshold of flexion velocity
P.HipStdDiffAngle_Thre = 1000;       % Threshold of std of hip diff angle
P.TorqueDeri_Thre = 20;              % Threshold of std of torque deriviation
% Ratio tolerance related to hip angle for transition between standing and lowering&lifting
P.RatioTol = 0.2; 
% The selection flag of Flexion angle (IMU) info based or Hip joint angle
% (Potentiometer) info based: false-Flexion angle; true-Hip joint angle
P.AngleInfoFlag = false;                       


end