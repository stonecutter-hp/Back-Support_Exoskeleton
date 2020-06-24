function DesiredTorque = TorqueGenerate(mode,ConInf)
global P;
% calculate desired torque or impedence according to designed algorithm
%% necessary parameters for torque generation algorithm
mt = P.TrunkM;
ma = P.ArmM;
Lt = P.TrunkHalfL;
La = P.ArmL;
Ls = P.ShoulderHalfL;
Kg = P.GravityKg;
Kd = P.DynamicKd;
K = P.DynamicK;
MaxV = P.DynamicMaxV;
g = P.g;
AlphaMean = ConInf(1);
AlphaDot = ConInf(2);
Beta = abs(ConInf(3));

%% calculate T_g and T_f
% other motion no assist
if mode(1) == 1;
    Con = 0; 
else 
    Con = 1;
end
% Gravity compensation term
Tg = Con*Kg*(mt*g*Lt*sin(AlphaMean)+ma*g*(Lt*sin(AlphaMean)+La*cos(ALphaMean)));
% Friction compensation term
Tf = Con*0;
% Dynamic compensation term
if (mode(1) == 2 || mode(1) == 3)  % stastic holding
    Td = Con*0;
else
    Td = Con*Kd*(K-abs(AlphaDot)/MaxV)*AlphaDot*sin(AlphaMean);
end
% Force ratio
if (mode(1) == 1 || mode(1) == 2 || mode(1) == 6 || mode(1) == 7) % symmetric motion
    h = 1;
else
    h = sqrt(1+8*sin(AlphaMean)*sin(Beta)*Lt*Ls/(4*Lt^2+(2-2*cos(Beta))*Ls^2-4*sin(AlphaMean)*sin(Beta)*Lt*Ls));
end
if mode(2) == 1   % left asymmetric
    ratioL = 1/(1+h);  % ratio for left motor
    ratioR = h/(1+h);  % ratio for right motor
elseif mode(2) == 2  % right asymmetric
    ratioL = h/(1+h);  
    ratioR = 1/(1+h);
else
    ratioL = 0.5;
    ratioR = 0.5;
end
% Desired Torque
if Tg+Tf+Td < 0
    DesiredTorque = [0 0];
elseif Tg+Tf+Td >= 100
    DesiredTorque = [ratioL rationR]*99;
else
    DesiredTorque = [ratioL rationR]*(Tg+Tf+Td);   % left motor torque
end



end