function TimerStop(Ttimer,~)
% Stop the usart communication and timer
global ExoP;
global TempApp;
%% Delete the timer
delete(Ttimer);
clear Ttimer
% disp('Timer stopped!');
% Check if serial port error or not
if isvalid(ExoP.config{1,1})
    ProgStop();
end
%% Saving data
DataSaving();
outPutStatus(TempApp,'Data Saved.');
set(TempApp.btnRun,'value',0);
% Normal Auto Stop Flag
if(ExoP.stopFlag == 0)
    outPutStatus(TempApp,'Program Auto Stopped.');
end
end