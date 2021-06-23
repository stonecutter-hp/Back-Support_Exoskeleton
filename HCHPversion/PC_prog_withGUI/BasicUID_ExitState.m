function mode = BasicUID_ExitState(HipMeanAngle,HipDiffAngle,HipStdAngle,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres)
% Exit state processing
global ExoP;

if ( [ExoP.UID.AngleInfoFlag; ...
      HipMeanAngle < ExoP.UID.HipMeanAngle_Thre(1); ...
      abs(ExoP.HipDiffAngle(max(end-ConThres+1,1):end)) < ExoP.UID.HipDiffAngle_Thre(1); ...
      HipStdAngle < ExoP.UID.HipStdAngle_Thre(2)] )
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Standing;    
elseif ~ExoP.UID.AngleInfoFlag ...
   && Alpha < ExoP.UID.Alpha_Thre(1) ...
   && abs(AlphaDot) < ExoP.UID.AlphaDot_Thre(3)
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Standing;
else
    mode(1,1) = ExoP.NoTrans;
    mode(1,2) = ExoP.Exit;
end

end