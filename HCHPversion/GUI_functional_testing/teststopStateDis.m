function teststopStateDis()
% Display stop state
% Program stop Flag: 0-normal auto stop; 1-normal manually stop
%                    2-cannot open serial port; 3-handshake error; 
%                    4-serial communication error
global testP;
global testTempApp;

% Normal Auto Stop Flag
if(testP.stopFlag == 0)
    outPutStatus(testTempApp,'Program Auto Stopped.');
elseif(testP.stopFlag == 1)
    outPutStatus(testTempApp,'Program Manually Stopped.');
% elseif(testP.stopFlag == 2)
%     outPutStatus(testTempApp,'Error Stopped: Fail Port Open!');
% elseif(testP.stopFlag == 3)
%     outPutStatus(testTempApp,'Error Stopped: Fail Handshake!');
% elseif(testP.stopFlag == 4)
%     outPutStatus(testTempApp,'Error Stopped: Fail Communication!');
end
set(testTempApp.statebtnRun,'value',0);

end