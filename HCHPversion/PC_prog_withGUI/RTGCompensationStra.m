function DesiredTorque = RTGCompensationStra(mode)
global ExoP;
% This function is to generate reference torque for both side of torque
% transmission system according to compensation strategy
DesiredTorque = zeros(1,2);

switch mode(1,2)
    case {ExoP.Lowering, ExoP.Grasping, ExoP.Lifting}
        % Generate desired torque according to the Impedance strategy:
        % Kg*M*g*sin(alpha)
        DesiredTorque = [0.5, 0.5]*(ExoP.RTG.GravityKg*ExoP.Subject.TrunkM*ExoP.g)*sin((ExoP.angleP-ExoP.TrunkAngleT0)*ExoP.d2r);
    otherwise
        DesiredTorque = zeros(1,2);
end


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