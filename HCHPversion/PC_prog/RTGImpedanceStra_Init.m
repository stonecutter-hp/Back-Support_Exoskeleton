function P = RTGImpedanceStra_Init()
% Initialize the specific parameters of Impedance RTG strategy
% Attention the unity of unit rad/deg
P.Kp = 0.56; % 0.4;           % Nm/deg,   Rendered stiffness
P.Kv = 0;             % Nm*s/deg, Rendered damping
P.Alpha0 = 0;         % deg,      Virtual alpha0
P.Alphadot0 = 0;      % deg/s,    Virtual alpha0 dot

end