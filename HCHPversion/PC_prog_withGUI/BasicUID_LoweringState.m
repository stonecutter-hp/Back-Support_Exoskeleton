function mode = BasicUID_LoweringState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres)
% Lowering state processing
global ExoP;
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
   & ((PeakFlag && HipMeanAngle > ExoP.UID.HipMeanAngle_Thre(3) && HipStdAngle < ExoP.UID.HipStdAngle_Thre(4)) ...
   | ExoP.HipStdAngle(max(end-ConThres+1,1):end) < ExoP.UID.HipStdAngle_Thre(2))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Grasping;   
elseif ~ExoP.UID.AngleInfoFlag ...
       & ((PeakFlag && Alpha > ExoP.UID.Alpha_Thre(2) && abs(AlphaDot) < ExoP.UID.AlphaDot_Thre(2)) ...
       | abs(ExoP.adotPV(max(end-ConThres+1,1):end)) < ExoP.UID.AlphaDot_Thre(3))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Grasping;     
elseif ExoP.UID.AngleInfoFlag ...
       && ((HipMeanAngle > (1+ExoP.UID.RatioTol(1))*ExoP.UID.HipMeanAngle_Thre(2) ... 
       && HipStdAngle > ExoP.UID.HipStdAngle_Thre(4) && HipStdAngleSign < 0) ...
       || HipMeanAngle < (1-ExoP.UID.RatioTol(2))*ExoP.UID.HipMeanAngle_Thre(3))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Lifting;    
elseif ~ExoP.UID.AngleInfoFlag ...
       && (Alpha > (1+ExoP.UID.RatioTol(1))*ExoP.UID.Alpha_Thre(1) && AlphaDot < -ExoP.UID.AlphaDot_Thre(2) ...
       || Alpha < (1-ExoP.UID.RatioTol(2))*ExoP.UID.Alpha_Thre(2))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Lifting;    
else
    mode(1,1) = ExoP.NoTrans;
    mode(1,2) = ExoP.Lowering;     
end

%% Keep updating the peak value of trunk flexion angle
if ExoP.UID.AngleInfoFlag
    % Updating peak value
    if ExoP.HipMeanAngle(end) > ExoP.TrunkAnglePeak  
        ExoP.TrunkAngleLeftPeak = ExoP.angleAL(end);
        ExoP.TrunkAngleRightPeak = ExoP.angleAR(end);
        ExoP.TrunkAnglePeak = (ExoP.TrunkAngleLeftPeak + ExoP.TrunkAngleRightPeak)/2;
    end  
else
    % Updating peak value
    if ExoP.angleP(end) > ExoP.TrunkAnglePeak  
        ExoP.TrunkAnglePeak = ExoP.angleP(end);
        ExoP.TrunkAngleLeftPeak = ExoP.TrunkAnglePeak/2;
        ExoP.TrunkAngleRightPeak = ExoP.TrunkAnglePeak/2;        
    end   
end


end