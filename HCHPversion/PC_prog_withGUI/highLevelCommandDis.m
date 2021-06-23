function highLevelCommandDis()
% Display stop state
% Program stop Flag: 0-normal auto stop; 1-normal manually stop
%                    2-cannot open serial port; 3-handshake error; 
%                    4-serial communication error
global ExoP;
global TempApp;

if ExoP.MotionMode(end,1) == ExoP.StateTrans
   TempApp.txtMode.Value = ['Last State: ',ExoP.MotionModeDis(ExoP.MotionMode(end-1,2)),...
                             10,'Curr State: ',ExoP.MotionModeDis(ExoP.MotionMode(end,2)),...
                             10,'Cycles: ',num2str(ExoP.BendCycle)];
end

end