function mode = BasicUID_GraspingState(HipMeanAngle,HipDiffAngle,HipStdAngle,Alpha,AlphaDot,Beta,HipStdDiffAngle,ConThres)
% Grasping state processing
global ExoP;
%% Keep updating the peak value of trunk flexion angle
if size(ExoP.HipMeanAngle,1) >= 3
    if ExoP.UID.AngleInfoFlag ...
       && ExoP.HipMeanAngle(end-1) > ExoP.HipMeanAngle(end-2) ...
       && ExoP.HipMeanAngle(end-1) > ExoP.HipMeanAngle(end)
        % Keep updating peak value
        if ExoP.HipMeanAngle(end-1) > ExoP.TrunkAnglePeak  
            ExoP.TrunkAngleLeftPeak = ExoP.angleAL(end-1);
            ExoP.TrunkAngleRightPeak = ExoP.angleAR(end-1);
            ExoP.TrunkAnglePeak = (ExoP.TrunkAngleLeftPeak + ExoP.TrunkAngleRightPeak)/2;
        end
    elseif ~ExoP.UID.AngleInfoFlag ...
           && ExoP.angleP(end-1) > ExoP.angleP(end-2) ...
           && ExoP.angleP(end-1) > ExoP.angleP(end)
        % Keep updating peak value
        if ExoP.angleP(end-1) > ExoP.TrunkAnglePeak  
            ExoP.TrunkAnglePeak = ExoP.angleP(end-1);
            ExoP.TrunkAngleLeftPeak = ExoP.TrunkAnglePeak/2;
            ExoP.TrunkAngleRightPeak = ExoP.TrunkAnglePeak/2;        
        end  
    end
end

%% Transition condition
if ExoP.UID.AngleInfoFlag && HipStdAngle > ExoP.UID.HipStdAngle_Thre(4)
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Lifting;    
elseif ~ExoP.UID.AngleInfoFlag && AlphaDot < ExoP.UID.AlphaDot_Thre(2)
    mode(1,1) = ExoP.StateTrans;
    mode(1,2) = ExoP.Lifting;     
else
    mode(1,1) = ExoP.NoTrans;
    mode(1,2) = ExoP.Grasping;    
end
    
end





