function P = RTGVelBasedStra_Init()
% Initialize the specific parameters of Velocity-based Compensation RTG
% strategy 
% Attention the unity of unit rad/deg
% Detialed torque generation strategy can refer to P2C

P.GravityKg = 0.3;    % Gravity compensation level coefficient
P.DynamicKd = 20;     % Adjustable dynamic compensation level coeffiecint, here fixed
P.DynamicK = 10;      % Adjustable auxiliary coefficient for dynamic compensation, here fixed 
P.DynamicVmax = 100;  % Adjusted assistive timing coefficient, here fixed
end