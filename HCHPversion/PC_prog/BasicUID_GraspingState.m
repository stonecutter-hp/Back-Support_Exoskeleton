function mode = BasicUID_GraspingState(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres)
% Grasping state processing
global ExoP;

%% Transition condition
if ExoP.UID.AngleInfoFlag ...
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
    mode(1,2) = ExoP.Grasping;    
end

%% Keep updating the peak value of trunk flexion angle
if ExoP.UID.AngleInfoFlag
    % Keep updating peak value
    if ExoP.HipMeanAngle(end) > ExoP.TrunkAnglePeak  
        ExoP.TrunkAngleLeftPeak = ExoP.angleAL(end);
        ExoP.TrunkAngleRightPeak = ExoP.angleAR(end);
        ExoP.TrunkAnglePeak = (ExoP.TrunkAngleLeftPeak + ExoP.TrunkAngleRightPeak)/2;
    end
else
    % Keep updating peak value
    if ExoP.angleP(end) > ExoP.TrunkAnglePeak  
        ExoP.TrunkAnglePeak = ExoP.angleP(end);
        ExoP.TrunkAngleLeftPeak = ExoP.TrunkAnglePeak/2;
        ExoP.TrunkAngleRightPeak = ExoP.TrunkAnglePeak/2;        
    end  
end


end





