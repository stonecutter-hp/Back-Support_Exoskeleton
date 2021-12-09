function SerialPorts_Init(McuPort)
global P;
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
P.config{1,1} = McuSerial;
% try to open the assigned MCU&PC communication port
try
    fopen(McuSerial);
    Transtate = fscanf(McuSerial); 
    disp('MCU COM opened !');
catch
    msgbox('Can not open MCU COM !');
    return
end

% Wait for correct handshake ready signal
% as the MCU should keep sending ReadyFlag  
while strcmp(Transtate,P.ReadyFlag)
    Transtate = fscanf(McuSerial);
end

end