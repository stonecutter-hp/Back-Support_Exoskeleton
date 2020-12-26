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
McuSerial.BaudRate = 115200;
McuSerial.InputBufferSize = 512;
McuSerial.OutputBufferSize = 512;
McuSerial.Terminator = 'CR/LF';
McuSerial.DataTerminalReady='on';
McuSerial.RequestToSend='off';

% McuSerial.BytesAvailableFcnMode = 'terminator';    %中断触发事件
% McuSerial.BytesAvailableFcnCount = 6;              %当缓冲区数据
% McuSerial.BytesAvailableFcn = @ReceiveData;        %调用回调函数
% store the serial port configuration
P.config{1,1} = McuSerial;
% try to open the assigned MCU&PC communication port
try
    fopen(McuSerial);
    fscanf(McuSerial);   % as the MCU keep sending data
    disp('MCU COM opened !');
catch
    msgbox('Can not open MCU COM !');
    return
end


end