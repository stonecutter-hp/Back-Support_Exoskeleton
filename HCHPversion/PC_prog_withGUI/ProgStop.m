function ProgStop()
% Program Stop Processing Function
% Mainly send stop command and close the serial port
global ExoP;
global TempApp;
%% Send command for stop status to low-level controller
McuSerial = ExoP.config{1,1};
% Stop state
TransState = 'TL0000TR0000M00';
flushoutput(McuSerial);           % flush the output buffer
fprintf(McuSerial,TransState);    % send the stopped commnad
% wait until data are all sent
while McuSerial.BytesToOutput ~= 0  
end

% Make sure low-level controller recieved the s
Transtate1 = fscanf(McuSerial);
while ~strcmp(Transtate1,ExoP.ReadyFlag) && ~strcmp(Transtate1,ExoP.NotReadyFlag)
%     flushinput(McuSerial);
    Transtate1 = fscanf(McuSerial);
    fprintf(McuSerial,TransState);    % keep send the stopped commnad
end
TempApp.txtMode.Value = ['Last State: ',ExoP.MotionModeDis(ExoP.MotionMode(end,2)),...
                          10,'Curr State: Stop',...
                          10,'Cycels: ',num2str(ExoP.BendCycle)];
%% Stop the serial port 
% stopasync(McuPort);
fclose(McuSerial);
delete(McuSerial);
% disp('Serial port shut down!')
outPutStatus(TempApp,'Serial Port Shut Down.');
pause(5/1000);
end