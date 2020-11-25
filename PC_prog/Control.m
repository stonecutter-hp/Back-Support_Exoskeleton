function Control()
global P;
%% Conducting motion detection and desired torque generation 
[mode,ConInf] = MotionDetection();
DesiredTorque = TorqueGenerate(mode,ConInf);

%% Saving necessary information
% Including mode, DesiredTorque(for low-level control) or even some
% detection referred information (saving for further analysis)

% mode is 2*1 vector: mode(1) refer to motion type, mode(2) refer to left/right asymmetric
P.MotionMode = [P.MotionMode,mode];           
% DesiredTorque is 2*1 vector: (1) for left, (2) for right
P.DesiredTorque = [P.DesiredTorque,DesiredTorque]; 
end