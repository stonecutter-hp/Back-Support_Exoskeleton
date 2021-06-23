function DesiredTorque = RTGVelBasedStra(mode)
global ExoP;
% This function is to generate reference torque for both side of torque
% transmission system according to the velocty-based compensation strategy
% The detailed strategy can be referred to the P2C
DesiredTorque = zeros(1,2);

% Ensure reasonable reference torque
if DesiredTorque(1) < ExoP.RTGLowerBound
    DesiredTorque(1) = ExoP.RTGLowerBound;
elseif DesiredTorque(1) > ExoP.RTGUpperBound
    DesiredTorque(1) = ExoP.RTGUpperBound;
end

if DesiredTorque(2) < ExoP.RTGLowerBound
    DesiredTorque(2) = ExoP.RTGLowerBound;
elseif DesiredTorque(2) > ExoP.RTGUpperBound
    DesiredTorque(2) = ExoP.RTGUpperBound;
end
end