function stopStateDis()
% Display stop state
% Program stop Flag: 0-normal auto stop; 1-normal manually stop
%                    2-cannot open serial port; 3-handshake error; 
%                    4-serial communication error
global ExoP;
global TempApp;

% Normal Auto Stop Flag
if(ExoP.stopFlag == 0)
    outPutStatus(TempApp,'Program Auto Stopped.');
elseif(ExoP.stopFlag == 1)
    outPutStatus(TempApp,'Program Manually Stopped.');
elseif(ExoP.stopFlag == 2)
    outPutStatus(TempApp,'Error Stopped: Fail Port Open!');
elseif(ExoP.stopFlag == 3)
    outPutStatus(TempApp,'Error Stopped: Fail Handshake!');
elseif(ExoP.stopFlag == 4)
    outPutStatus(TempApp,'Error Stopped: Fail Communication!');
elseif(ExoP.stopFlag == 5)
    outPutStatus(TempApp,'Error Stopped: Timer Callback Error!');
end
set(TempApp.btnRun,'value',0);

end