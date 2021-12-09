function mode = VisualCueUIDtest()
% This function is to change the Visual Cue according to certain experiment
% protocol for UID strategy testing
global ExoP;

if ExoP.VCClickFlag 
    switch ExoP.VCStatus
        case ExoP.Standing
            ExoP.VCStatus = ExoP.Lowering;
        case ExoP.Lowering
            ExoP.VCStatus = ExoP.Grasping;
        case ExoP.Grasping
            ExoP.VCStatus = ExoP.Lifting;
        case ExoP.Lifting
            ExoP.VCStatus = ExoP.Standing;
            ExoP.BendCycle = ExoP.BendCycle+1;
        otherwise
            ExoP.VCStatus = ExoP.Standing;
    end
    mode = [ExoP.StateTrans, ExoP.Exit, ExoP.None+ExoP.fricEnable*ExoP.fricCompen];
    ExoP.VCClickFlag = 0;
else
    mode = [ExoP.NoTrans, ExoP.Exit, ExoP.None+ExoP.fricEnable*ExoP.fricCompen];
end

end