function mode = BasicUID_StandingState(HipMeanAngle,HipDiffAngle,HipStdAngle,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres)
global ExoP;
% Standing state processing
if ( [ExoP.UID.AngleInfoFlag; ...
      HipMeanAngle < ExoP.UID.HipMeanAngle_Thre(1); ...
      abs(ExoP.HipDiffAngle(max(end-ConThres+1,1):end)) > ExoP.UID.HipDiffAngle_Thre(1); ...
      HipStdAngle > ExoP.UID.HipStdAngle_Thre(1)] )
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Walking;
elseif ExoP.UID.AngleInfoFlag ...
       && ((HipMeanAngle > ExoP.UID.HipMeanAngle_Thre(2) ...
       && HipStdAngle > ExoP.UID.HipStdAngle_Thre(3)) ...
        || HipMeanAngle > (1+ExoP.UID.RatioTol)*ExoP.UID.HipMeanAngle_Thre(2))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Lowering;
    ExoP.TrunkAngleLeftT0 = ExoP.angleAL(end);
    ExoP.TrunkAngleRightT0 = ExoP.angleAR(end);
    ExoP.TrunkAngleT0 = (ExoP.TrunkAngleLeftT0 + ExoP.TrunkAngleRightT0)/2;
elseif ~ExoP.UID.AngleInfoFlag ...
        && ((Alpha > ExoP.UID.Alpha_Thre(1) ...
        && AlphaDot > ExoP.UID.AlphaDot_Thre(1)) ...
        || Alpha > (1+ExoP.UID.RatioTol)*ExoP.UID.Alpha_Thre(1))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Lowering;
    ExoP.TrunkAngleT0 = Alpha;
    ExoP.TrunkAngleLeftT0 = ExoP.TrunkAngleT0/2;
    ExoP.TrunkAngleRightT0 = ExoP.TrunkAngleT0/2;
else
    mode(1,1) = ExoP.NoTrans;
    mode(1,2) = ExoP.Standing;    
end

end