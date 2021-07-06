function P = BasicUIDStrategy_Init()
% Initialize the specific parameters of the basic UID strategy
%% Thresholds of basic UID strategy 
P.HipMeanAngle_Thre = [0, 0, 0];     % Threshold of hip mean angle
P.HipDiffAngle_Thre = 0;             % Threshold of hip diff angle
P.HipStdAngle_Thre = [0, 0, 0, 0];   % Threshold of std of hip mean angle
P.Alpha_Thre = [10, 65];               % Threshold of flexion angle
P.AlphaDot_Thre = [10, 8, 8];         % Threshold of flexion velocity
P.HipStdDiffAngle_Thre = 1000;       % Threshold of std of hip diff angle
P.TorqueDeri_Thre = 20;              % Threshold of std of torque deriviation
% Ratio tolerance related to hip angle for transition between standing and lowering&lifting
P.RatioTol = [3.5, 0.4, 0.5]; 
% The selection flag of Flexion angle (IMU) info based or Hip joint angle
% (Potentiometer) info based: false-Flexion angle; true-Hip joint angle
P.AngleInfoFlag = false;                       

% Refers to Remarks 8 of this UID in the Thoughs Keeping NoteBook
if ((1+P.RatioTol(1))*P.HipMeanAngle_Thre(2) <= (1-P.RatioTol(2))*P.HipMeanAngle_Thre(3))
    P.HipMeanAngle_Thre(3) = (1+P.RatioTol(1))*P.HipMeanAngle_Thre(2)/(1-P.RatioTol(2)) - 5;
end
if ((1+P.RatioTol(1))*P.Alpha_Thre(1) <= (1-P.RatioTol(2))*P.Alpha_Thre(2))
    P.Alpha_Thre(2) = (1+P.RatioTol(1))*P.Alpha_Thre(1)/(1-P.RatioTol(2)) - 5;
end


end