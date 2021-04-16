function DesiredTorque = TorqueGenerate(mode,ConInf)
global P;
% calculate desired torque or impedence according to designed algorithm
% Refer to Presentation_20200828
%% necessary parameters for torque generation algorithm
mt = P.TrunkM;
ma = P.ArmM;
Lt = P.TrunkHalfL;
La = P.ArmL;
Ls = P.ShoulderHalfL;
Kg = P.GravityKg;
Kd = P.DynamicKd;
K = P.DynamicK;
MaxV = P.DynamicVmax;
g = P.g;
Alpha = ConInf(1)*P.d2r;                % rad
AlphaDot = ConInf(2)*P.d2r;             % rad/s
Beta = abs(ConInf(3))*P.d2r;            % rad
% other motion no assist
if mode(1) == 1
    Con = 0; 
else 
    Con = 1;
end
%% Torque generation based on biomechancial model
% % Gravity compensation term Tg
% Tg = Con*Kg*(mt+ma)*g*Lt*sin(Alpha);
% % Friction compensation term
% Tf = Con*0;
% % Dynamic compensation term
% if (mode(1) == 2 || mode(1) == 3)  % stastic holding
%     Td = Con*0;
% else
%     Td = Con*Kd*K(abs(AlphaDot)/MaxV-2)*AlphaDot*sin(Alpha);
% end
% % Force ratio
% if (mode(1) == 1 || mode(1) == 2 || mode(1) == 6 || mode(1) == 7) % symmetric motion
%     h = 1;
% else
%     h = sqrt(1+8*sin(Alpha)*sin(Beta)*Lt*Ls/(4*Lt^2+(2-2*cos(Beta))*Ls^2-4*sin(Alpha)*sin(Beta)*Lt*Ls));
% end
% if mode(2) == 1        % left asymmetric
%     ratioL = 1/(1+h);  % ratio for left motor
%     ratioR = h/(1+h);  % ratio for right motor
% elseif mode(2) == 2    % right asymmetric
%     ratioL = h/(1+h);  
%     ratioR = 1/(1+h);
% else
%     ratioL = 0.5;
%     ratioR = 0.5;
% end
% % Desired Torque
% if Tg+Tf+Td < 0
%     DesiredTorque = [0; 0];
% elseif Tg+Tf+Td >= 100
%     DesiredTorque = [ratioL; ratioR]*99;
% else
%     DesiredTorque = [ratioL; ratioR]*(Tg+Tf+Td);   % left motor torque
% end

%% Torque generation based on impedance regulation, mainly for testing
% Here Kp*(Alpha-Alpha0)+Kv*(AlphaDot-AlphaDot0) is for both side transmission systems
Kp = P.ImpedanceKp;
Kv = P.ImpedanceKv;
Alpha0 = P.VirAlpha0; 
AlphaDot0 = P.VirAlphadot0;
DesiredTorque = [0.5, 0.5]*(Kp*(Alpha-Alpha0)+Kv*(AlphaDot-AlphaDot0));

end