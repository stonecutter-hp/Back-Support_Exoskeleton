function SerialStop(SerialPort,~)
global ExoP;
ExoP.stopFlag = 3;
delete(SerialPort);
clear SerialPort;
disp('Serial port error shut down!')

% check if timer is started
if(~isempty(ExoP.config{2,1}) && isvalid(ExoP.config{2,1}))
    ExoP.stopFlag = 4;
    stop(ExoP.config{2,1});
else
    %% Saving data
    ExoP.stopFlag = 3;
    stopStateDis();
    DataSaving();
end

end