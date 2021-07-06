function DesiredTorque = RTGImpedanceStra(mode)
global ExoP;
% This program is to generate reference torque for both side of the torque
% transmission system according to the Impedance RTG strategy
DesiredTorque = zeros(1,2);

switch mode(1,2)
    case {ExoP.Lowering, ExoP.Grasping, ExoP.Lifting}
        % Generate desired torque according to the Impedance strategy
%         DesiredTorque = [0.5, 0.5]*(ExoP.RTG.Kp*(ExoP.angleP-ExoP.RTG.Alpha0)...
%                                   + ExoP.RTG.Kv*(ExoP.adotPV-ExoP.RTG.Alphadot0));
        DesiredTorque = [0.5, 0.5]*(ExoP.RTG.Kp*(ExoP.angleP(end)-ExoP.TrunkAngleT0-ExoP.RTG.Alpha0)...
                                    + ExoP.RTG.Kv*(ExoP.adotPV(end)-ExoP.RTG.Alphadot0));                              
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