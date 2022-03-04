function P = HongPengData()
% The biomechanical data of HongPeng
% Data set refers to the P1C draft
P.TrunkM = 41;           % unit: kg
P.ArmM = 5;              % unit: kg
P.TrunkHalfL = 0.26;     % unit: m
P.ShoulderHalfL = 0.155; % unit: m
P.ArmL = 0.24;           % unit: m
P.Df = 0.05;             % moment arm for flexion spinal force
P.Dt = 0.05;             % moment arm for twisting spinal force
P.con = 1.02;            % co-constration index

% Support beam length: 75%*back length
P.SupportBeamL = 2*P.TrunkHalfL*0.75;
end