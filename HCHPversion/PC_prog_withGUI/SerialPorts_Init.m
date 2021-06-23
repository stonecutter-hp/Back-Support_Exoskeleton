function SerialPorts_Init(McuPort)
global ExoP;
global TempApp;
delete(instrfindall('Type','serial'));         % delete the last configuration for serial ports

% find if there have serial port for MCU port
McuSerial = instrfind('Type','serial','Port',McuPort,'Tag','');  
if isempty(McuSerial)                          % if there do not have one
    McuSerial = serial(McuPort);               % create one
else                                           % if there exists one
    fclose(McuSerial);                         
    McuSerial = McuSerial(1);                  % use the first one
end
% set the properties of the serial port
McuSerial.BaudRate = 460800;
McuSerial.InputBufferSize = 512;
McuSerial.OutputBufferSize = 512;
% Notice when use fprintf(obj,'cmd') for data sending, the default format
% is %s\n and '\n' will be automatically replaced by assigned terminator
McuSerial.Terminator = 'CR/LF';
McuSerial.DataTerminalReady='on';
McuSerial.RequestToSend='off';
McuSerial.ErrorFcn = @SerialStop;
% % ------------ Receiving interruption configuration ------------- %
% % Specify if the bytes-available event is generated after a specified
% % number of bytes is available in the input buffer, or after a terminator
% % is read  
% McuSerial.BytesAvailableFcnMode = 'terminator';    
% % Specify the number of bytes that must be available in the input buffer to
% % generate a bytes-available event 
% McuSerial.BytesAvailableFcnCount = 6;
% % Specify the callback function to execute when a specified number of bytes
% % is available in the input buffer, or a terminator is read 
% McuSerial.BytesAvailableFcn = @ReceiveData;

% store the serial port configuration
ExoP.config{1,1} = McuSerial;
% try to open the assigned MCU&PC communication port
try
    fopen(McuSerial);
    fscanf(McuSerial); 
    outPutStatus(TempApp,'Serial Port Opened.');
catch
    outPutStatus(TempApp,'Serial Port Cannot Open!');
    msgbox('Can not open Serial Port !');
    ExoP.stopFlag = 2;
    stopStateDis();
    return
end

% Wait for correct handshake ready signal
% as the MCU should keep sending ReadyFlag 
outPutStatus(TempApp,'Wait for Handshake...');
pause(5/1000);
flushinput(McuSerial);
Transtate = fscanf(McuSerial);
tic
while ~strcmp(Transtate,ExoP.ReadyFlag)
%     flushinput(McuSerial);
    Transtate = fscanf(McuSerial);
    if(toc > 30)
        ExoP.stopFlag = 3;
        fclose(McuSerial);
        delete(McuSerial);
        DataSaving();
        stopStateDis();
        return
    end
end

% Notice that the initial command send to low-level controller during
% handshake process indicates that mode = 1 and side = 0
TransState = 'TL0000TR0000M10';
fprintf(McuSerial,TransState);    % send the data
flushoutput(McuSerial);           % flush the output buffer
outPutStatus(TempApp,'Sucessful Handshake.');
TempApp.txtMode.Value = ['Last State: Exit',10,'Curr State: Exit',10,'Cycels: 0'];
pause(5/1000);
end