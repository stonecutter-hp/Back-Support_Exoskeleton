function Control()
global ExoP;
%% Conducting motion detection and desired torque generation 
[mode,ConInf] = MotionDetection();
DesiredTorque = TorqueGenerate(mode,ConInf);

%% Saving necessary information
% Including mode, DesiredTorque(for low-level control) or even some
% detection referred information (saving for further analysis)

% mode is 1*2 vector: mode(1) refer to motion type, mode(2) refer to left/right asymmetric
ExoP.MotionMode = [ExoP.MotionMode;mode];           
% DesiredTorque is 1*2 vector: (1) for left, (2) for right
ExoP.DesiredTorque = [ExoP.DesiredTorque;DesiredTorque]; 
end