function SerialStop(SerialPort,~)
global ExoP;
global TempApp;
fclose(SerialPort);
delete(SerialPort);
% disp('Serial port shut down!')
outPutStatus(TempApp,'Serial Port Error!');
pause(5/1000);
% check if timer is started
if(~isempty(ExoP.config{2,1}) && isvalid(ExoP.config{2,1}))
    outPutStatus(TempApp,'Communication Fail!');
    pause(5/1000);
    stop(ExoP.config{2,1});
else
    %% Saving data
    outPutStatus(TempApp,'Handshake Fail!');
    pause(5/1000);
    DataSaving();
    outPutStatus(TempApp,'Data Saved.');
end

set(TempApp.btnRun,'value',0);
outPutStatus(TempApp,'Program Accidently Stopped!');
end