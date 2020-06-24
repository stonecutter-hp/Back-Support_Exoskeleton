function Control()
global P;
%% Conducting motion detection and desired torque generation 
[mode,ConInf] = MotionDetection();
DesiredTorque = TorqueGenerate(mode,ConInf);

%% Saving necessary information
% Including mode, DesiredTorque(for low-level control) or even some
% detection referred information (saving for further analysis)
P.MotionMode = [P.MotionMode;mode];   % mode is 1*2 vector: mode(1) refer to motion type, mode(2) refer to left/right asymmetric
P.DesiredTorque = [P.DesiredTorque;DesiredTorque]; % DesiredTorque is 1*2 vector: (1) for left, (2) for right
% P.AngleMean = [P.AngleMean,ConInf(1)];

end