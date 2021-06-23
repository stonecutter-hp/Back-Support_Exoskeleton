function mode = BasicUID_LoweringState(HipMeanAngle,HipDiffAngle,HipStdAngle,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres)
global ExoP;
% Lowering state processing
%% Detect the peak value of trunk flexion angle
PeakFlag = false;
if size(ExoP.HipMeanAngle,1) < 3
    PeakFlag = false;   % Indicate not find
elseif ExoP.UID.AngleInfoFlag ...
       && ExoP.HipMeanAngle(end-1) > ExoP.HipMeanAngle(end-2) ...
       && ExoP.HipMeanAngle(end-1) > ExoP.HipMeanAngle(end)
    PeakFlag = true;
elseif ~ExoP.UID.AngleInfoFlag ...
       && ExoP.angleP(end-1) > ExoP.angleP(end-2) ...
       && ExoP.angleP(end-1) > ExoP.angleP(end)
    PeakFlag = true;
end

%% Transition condition
if ExoP.UID.AngleInfoFlag ...
   && ((PeakFlag && HipStdAngle < ExoP.UID.HipStdAngle_Thre(4)) ...
   || HipMeanAngle > (1+ExoP.UID.RatioTol)*ExoP.UID.HipMeanAngle_Thre(3))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Grasping; 
    % Updating peak value
    if ExoP.HipMeanAngle(end-1) > ExoP.TrunkAnglePeak  
        ExoP.TrunkAngleLeftPeak = ExoP.angleAL(end-1);
        ExoP.TrunkAngleRightPeak = ExoP.angleAR(end-1);
        ExoP.TrunkAnglePeak = (ExoP.TrunkAngleLeftPeak + ExoP.TrunkAngleRightPeak)/2;
    end    
elseif ~ExoP.UID.AngleInfoFlag ...
       && ((PeakFlag && abs(AlphaDot) < ExoP.UID.AlphaDot_Thre(2)) ...
       || Alpha > (1+ExoP.UID.RatioTol)*ExoP.UID.Alpha_Thre(2))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Grasping;    
    % Updating peak value
    if ExoP.angleP(end-1) > ExoP.TrunkAnglePeak  
        ExoP.TrunkAnglePeak = ExoP.angleP(end-1);
        ExoP.TrunkAngleLeftPeak = ExoP.TrunkAnglePeak/2;
        ExoP.TrunkAngleRightPeak = ExoP.TrunkAnglePeak/2;        
    end    
else
    mode(1,1) = ExoP.NoTrans;
    mode(1,2) = ExoP.Lowering;     
end

end