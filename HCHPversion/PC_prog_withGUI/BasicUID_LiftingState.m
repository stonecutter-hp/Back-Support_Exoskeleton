function mode = BasicUID_LiftingState(HipMeanAngle,HipDiffAngle,HipStdAngle,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres)
global ExoP;
% Lifting state processing
if ExoP.UID.AngleInfoFlag ...
   && ((HipMeanAngle < ExoP.UID.HipMeanAngle_Thre(2) ...
   && HipStdAngle < ExoP.UID.HipStdAngle_Thre(2)) ...
   || HipMeanAngle < (1-ExoP.UID.RatioTol)*ExoP.UID.HipMeanAngle_Thre(2))
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Standing;
elseif ~ExoP.UID.AngleInfoFlag ...
        && ((Alpha < ExoP.UID.Alpha_Thre(1) ...
        && AlphaDot < ExoP.UID.AlphaDot_Thre(3)) ...
        || Alpha < (1-ExoP.UID.RatioTol)*ExoP.UID.Alpha_Thre(1))    
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Standing;       
    ExoP.BendCycle = ExoP.BendCycle+1;
else
    mode(1,1) = ExoP.NoTrans;
    mode(1,2) = ExoP.Lifting;      
end

end