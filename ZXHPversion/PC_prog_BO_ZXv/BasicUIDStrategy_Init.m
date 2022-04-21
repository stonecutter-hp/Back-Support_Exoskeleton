function P = BasicUIDStrategy_Init()
% Initialize the specific parameters of the basic UID strategy
%% Thresholds of basic UID strategy 
P.HipMeanAngle_Thre = [0, 0, 0];     % Threshold of hip mean angle
P.HipDiffAngle_Thre = 0;             % Threshold of hip diff angle
P.HipStdAngle_Thre = [0, 0, 0, 0];   % Threshold of std of hip mean angle
P.Alpha_Thre = [16, 30];             % Threshold of flexion angle
P.AlphaDot_Thre = [20, 14, 8];       % Threshold of flexion velocity
P.RatioTol = [0.25, 0.28, 0.5]; 
% P.RatioTol = [0.2, 0.2, 0.3];
P.HipStdDiffAngle_Thre = 1000;       % Threshold of std of hip diff angle
P.TorqueDeri_Thre = 20;              % Threshold of std of torque deriviation

%% Make sure the reasonable of threshold setting (Trunk Velocity Version)
if (1+P.RatioTol(1))*P.Alpha_Thre(1) < P.Alpha_Thre(1)
    P.RatioTol(1) = 0.1;
end
if P.Alpha_Thre(1) > P.Alpha_Thre(2)
    P.Alpha_Thre(2) = P.Alpha_Thre(1)+10;
end
if P.AlphaDot_Thre(2) > P.AlphaDot_Thre(1)
    P.AlphaDot_Thre(1) = P.AlphaDot_Thre(2);
end
if P.AlphaDot_Thre(2) < P.AlphaDot_Thre(3)
    P.AlphaDot_Thre(3) = P.AlphaDot_Thre(2);
end
if P.Alpha_Thre(1) <= (1-P.RatioTol(2))*P.Alpha_Thre(2)
    P.RatioTol(2) = 1-P.Alpha_Thre(1)/P.Alpha_Thre(2)+0.1;
end
if (1-P.RatioTol(3))*P.Alpha_Thre(1) >= (1-P.RatioTol(2))*P.Alpha_Thre(2)
    P.RatioTol(3) = 1-(1-P.RatioTol(2))*P.Alpha_Thre(2)/P.Alpha_Thre(1)+0.1;
end

% Refers to (<</>>) condition of of this UID in the Thoughs Keeping NoteBook
P.Angle_ThreAux(1) = (1+P.RatioTol(1))*P.Alpha_Thre(1);
P.Angle_ThreAux(2) = (1-P.RatioTol(2))*P.Alpha_Thre(2);
P.Angle_ThreAux(3) = (1-P.RatioTol(3))*P.Alpha_Thre(1);

% The selection flag of Flexion angle (IMU) info based or Hip joint angle
% (Potentiometer) info based: false-Flexion angle; true-Hip joint angle
P.AngleInfoFlag = false; 

% % % % % % P.Angle_ThreAux = [20, 10, 5];       
% % % % % % % Ratio tolerance related to hip angle for transition between standing and lowering&lifting
% % % % % % P.RatioTol(1) = P.Angle_ThreAux(1)/P.Alpha_Thre(1)-1;
% % % % % % P.RatioTol(2) = 1 - P.Angle_ThreAux(2)/P.Alpha_Thre(2);
% % % % % % P.RatioTol(3) = 1- P.Angle_ThreAux(3)/P.Alpha_Thre(1);
% % % % % % % The selection flag of Flexion angle (IMU) info based or Hip joint angle
% % % % % % % (Potentiometer) info based: false-Flexion angle; true-Hip joint angle
% % % % % % P.AngleInfoFlag = false;                       
% % % % % % 
% % % % % % % Refers to Remarks 8 of this UID in the Thoughs Keeping NoteBook
% % % % % % if ((1+P.RatioTol(1))*P.HipMeanAngle_Thre(2) <= (1-P.RatioTol(2))*P.HipMeanAngle_Thre(3))
% % % % % %     P.HipMeanAngle_Thre(3) = (1+P.RatioTol(1))*P.HipMeanAngle_Thre(2)/(1-P.RatioTol(2)) - 5;
% % % % % % end
% % % % % % if ((1+P.RatioTol(1))*P.Alpha_Thre(1) <= (1-P.RatioTol(2))*P.Alpha_Thre(2))
% % % % % %     P.Alpha_Thre(2) = (1+P.RatioTol(1))*P.Alpha_Thre(1)/(1-P.RatioTol(2)) - 5;
% % % % % % end


end