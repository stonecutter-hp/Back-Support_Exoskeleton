function VisualCueUIDtest()
% This function is to change the Visual Cue according to certain experiment
% protocol for UID strategy testing
global ExoP;

if ExoP.VCClickFlag 
    switch ExoP.VCStatus
        case ExoP.Standing
            if ExoP.WalkingFlag == 0
                ExoP.VCStatus = ExoP.Walking;
                ExoP.WalkingFlag = 1;
            else
                ExoP.VCStatus = ExoP.Lowering;
            end
        case ExoP.Walking
            ExoP.VCStatus = ExoP.Standing;            
        case ExoP.Lowering
            ExoP.VCStatus = ExoP.Grasping;
        case ExoP.Grasping
            ExoP.VCStatus = ExoP.Lifting;
        case ExoP.Lifting
            ExoP.VCStatus = ExoP.Standing;
            ExoP.VCCBendCycle = ExoP.VCCBendCycle+1;
        otherwise
            ExoP.VCStatus = ExoP.Standing;
    end
    ExoP.VCClickFlag = 0;
%     mode = [ExoP.StateTrans, ExoP.Exit, ExoP.None+ExoP.asyEnable*ExoP.asyCompen];
    ExoP.VCClickHis = [ExoP.VCClickHis; 1];
else
    ExoP.VCClickHis = [ExoP.VCClickHis; 0];
end
ExoP.VCCMode = [ExoP.VCCMode;ExoP.VCStatus];

end