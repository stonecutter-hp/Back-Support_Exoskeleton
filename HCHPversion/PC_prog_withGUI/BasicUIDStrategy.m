function mode = BasicUIDStrategy()
% This program is to conduct the basic UID strategy
% The basic UID strategy for HCHP prototype can refer to V1P in the
% Notebook 'Thoughts Keeping' in GOODNOTE APP 
% mode(1) - state transition flag
% mode(2) - motion state
% mode(3) - asymmetric direction
% ConInf - Alpha, AlphaDot, Beta feedback for reference torque generation
global ExoP;
% mode = [ExoP.NoTrans, ExoP.Exit, ExoP.None+ExoP.fricEnable*ExoP.fricCompen];
%% Calculate the utilized infomation for UID strategy
% The range of angle standard deviation
Range = ExoP.Range;

% The number of continous requirment satisfied items at once
ConThres = ExoP.ConThres;

% Mean angle of two hip joints
ExoP.HipMeanAngle = [ExoP.HipMeanAngle;(ExoP.angleAL(end)+ExoP.angleAR(end))/2];
HipMeanAngle = abs(ExoP.HipMeanAngle(end));   % Notice this is the absolute value

% Absolute value of the difference between two hip joints' angle
ExoP.HipDiffAngle = [ExoP.HipDiffAngle;ExoP.angleAL(end)-ExoP.angleAR(end)];
HipDiffAngle = abs(ExoP.HipDiffAngle(end));   % Notice this is the absolute value

% (Population) Standard deviation of HipMeanAngle and HipDiffAngle
HipStdAngle = std(ExoP.HipMeanAngle(max(1,end-Range+1):end),1);
HipStdDiffAngle = std(ExoP.HipDiffAngle(max(1,end-Range+1):end),1);
ExoP.HipStdAngle = [ExoP.HipStdAngle;HipStdAngle];
ExoP.HipStdDiffAngle = [ExoP.HipStdDiffAngle;HipStdDiffAngle];

% The sign of the Standard deviation of HipMeanAngle
HipStdAngleSign = sign(ExoP.HipMeanAngle(end) - ExoP.HipMeanAngle(max(1,end-Range+1)));

% Trunk flexion angle
Alpha = ExoP.angleP(end);

% Trunk flexion velocity directly from IMU
AlphaDot = ExoP.adotPV(end);

% Trunk twisting angle
Beta = ExoP.angleY(end);

% Last time's mode
Premode = ExoP.MotionMode(end,:);
%% Conduction basic UID strategy: Motion state detection (A simple FSM)
% Notive that mode(1,1) indicates the transition flag, mode(1,2)indicates
% motion state, mode(1,3) the bending side and friction compensation config
% of low-level controller.
switch Premode(1,2)
    case ExoP.Exit
        mode(1,1:2) = BasicUID_ExitState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,...
                                         Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres);
    case ExoP.Standing
        mode(1,1:2) = BasicUID_StandingState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,...
                                             Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres);
    case ExoP.Walking
        mode(1,1:2) = BasicUID_WalkingState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,...
                                            Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres);
    case ExoP.Lowering
        mode(1,1:2) = BasicUID_LoweringState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,...
                                             Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres); 
    case ExoP.Grasping
        mode(1,1:2) = BasicUID_GraspingState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,...
                                             Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres);    
    case ExoP.Lifting
        mode(1,1:2) = BasicUID_LiftingState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,...
                                            Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres);         
    otherwise
        mode(1,1:2) = [ExoP.NoTrans, ExoP.Exit];
end
% Exit condition detect for every loop
mode(1,1:2) = BasicUID_ExitCondition(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,...
                                     Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres,mode);

%% Conduction basic UID strategy: Asymmetric side classification and friction compensation selection
% Initial classification of left/right asymmetric. Notice that it also
% indicates if the friction compensation is enabled or not
if sign(Beta) == 0 
    mode(1,3) = ExoP.None + ExoP.fricEnable*ExoP.fricCompen;        % no aysmmetric
elseif sign(Beta) > 0
    mode(1,3) = ExoP.LeftAsy + ExoP.fricEnable*ExoP.fricCompen;     % left
else
    mode(1,3) = ExoP.RightAsy + ExoP.fricEnable*ExoP.fricCompen;    % right
end

%% Processing for certain state transition
if mode(1,1) == ExoP.StateTrans && (mode(1,2) == ExoP.Exit || mode(1,2) == ExoP.Standing)
    % Record the T0 value and peak value
    ExoP.T0Value = [ExoP.T0Value; [ExoP.TrunkAngleT0,ExoP.TrunkAngleLeftT0,ExoP.TrunkAngleRightT0]];
    ExoP.PeakValue = [ExoP.PeakValue; [ExoP.TrunkAnglePeak,ExoP.TrunkAngleLeftPeak,ExoP.TrunkAngleRightPeak]];
    if ExoP.UID.AngleInfoFlag
        % Reset Recorded T0 value and peack value
        ExoP.TrunkAngleLeftT0 = 0;
        ExoP.TrunkAngleRightT0 = 0;
        ExoP.TrunkAngleT0 = (ExoP.TrunkAngleLeftT0 + ExoP.TrunkAngleRightT0)/2;
        ExoP.TrunkAngleLeftPeak = 0;
        ExoP.TrunkAngleRightPeak = 0;
        ExoP.TrunkAnglePeak = (ExoP.TrunkAngleLeftPeak + ExoP.TrunkAngleRightPeak)/2;
    else
        % Reset Recorded T0 value and peack value
        ExoP.TrunkAngleT0 = 0;
        ExoP.TrunkAngleLeftT0 = ExoP.TrunkAngleT0/2;
        ExoP.TrunkAngleRightT0 = ExoP.TrunkAngleT0/2;
        ExoP.TrunkAnglePeak = 0;
        ExoP.TrunkAngleLeftPeak = ExoP.TrunkAnglePeak/2;
        ExoP.TrunkAngleRightPeak = ExoP.TrunkAnglePeak/2; 
    end
end




end