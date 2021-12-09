function testHighLevelDis()
% This function is to display certain info if UID and RTG are all set for
% testing purpose
global ExoP;
global TempApp;

if ExoP.MotionMode(end,1) == ExoP.StateTrans
   TempApp.txtMode.Value = ['Curr State: ',ExoP.MotionModeDis(ExoP.VCStatus),...
                            'Cycles: ',num2str(ExoP.BendCycle)];
end

end