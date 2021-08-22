function AmendMode = BasicUID_ExitCondition(HipMeanAngle,HipDiffAngle,HipStdAngle,HipStdAngleSign,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres,mode)
% Detect if the system triggered exit condition to guarantee the prototype
% operates under normal states
global ExoP;

if HipStdDiffAngle > ExoP.UID.HipStdDiffAngle_Thre
    AmendMode(1,1) = ExoP.StateTrans;
    AmendMode(1,2) = ExoP.Exit; 
    ExoP.BendCycle = 0; 
else
    % Keep unchanged
    AmendMode = mode(1,1:2);
end

end