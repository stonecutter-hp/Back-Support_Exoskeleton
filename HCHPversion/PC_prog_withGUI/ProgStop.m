function ProgStop()
% Program Stop Processing Function
% Mainly send stop command and close the serial port
global ExoP;
global TempApp;
%% Detect if the stop from the serialport failure
if ~isempty(find(seriallist("available") == ExoP.McuPort, 1))
    McuSerial = serialport(McuPort,460800);
    configureTerminator(McuSerial,"CR/LF");
    setDTR(McuSerial,true);
    setRTS(McuSerial,false);
    McuSerial.ErrorOccurredFcn = @SerialStop;
    % store the serial port configuration
    ExoP.config{1,1} = McuSerial; 
    ExoP.stopFlag = 4;
end
%% Send command for stop status to low-level controller
McuSerial = ExoP.config{1,1};
% Stop state
TransState = "TL0000TR0000M00";
writeline(McuSerial,TransState);    % send the stopped commnad

% Make sure low-level controller recieved the stop command
Transtate1 = char(readline(McuSerial));
while ~strcmp(Transtate1,ExoP.ReadyFlag) && ~strcmp(Transtate1,ExoP.NotReadyFlag)
    Transtate1 = char(readline(McuSerial));
    pause(1/1000);
    writeline(McuSerial,TransState);    % keep send the stopped commnad
end
TempApp.txtMode.Value = ['Last State: ',ExoP.MotionModeDis(ExoP.MotionMode(end,2)),...
                         'Curr State: Stop',...
                         'Cycels: ',num2str(ExoP.BendCycle)];
%% Stop the serial port 
delete(McuSerial);
clear McuSerial;
% disp('Serial port shut down!')
outPutStatus(TempApp,'Serial Port Shut Down.');
pause(5/1000);
end