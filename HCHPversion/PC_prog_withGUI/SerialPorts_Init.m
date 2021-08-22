function SerialPorts_Init(McuPort)
global ExoP;
global TempApp;
flag = true;  % To replace return

%% find if the selected serial port is available
p = find(seriallist("available") == McuPort);
if ~isempty(p)
    McuSerial = serialport(McuPort,460800);
else
    outPutStatus(TempApp,'Serial Port Cannot Open!');
    msgbox('Can not open Serial Port !');
    ExoP.stopFlag = 2;
    stopStateDis();
    flag = false;    
end

if flag
    %% set the properties of the serial port
    % Notice for serialport no need to configure the input/output buffer
    McuSerial.BaudRate = 460800;
    % Notice when use writeline(obj,"cmd") for data sending, the default format
    % is %s+terminator and the terminator will be automatically added at the
    % end of the cmd
    configureTerminator(McuSerial,"CR/LF");
    setDTR(McuSerial,true);
    setRTS(McuSerial,false);
    McuSerial.ErrorOccurredFcn = @SerialStop;
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

    %% try to open the assigned MCU&PC communication port
    try
        readline(McuSerial); 
        outPutStatus(TempApp,'Serial Port Opened.');
    catch
        outPutStatus(TempApp,'Serial Port Cannot Open!');
        msgbox('Can not open Serial Port !');
        ExoP.stopFlag = 2;
        delete(McuSerial);
        clear McuSerial;    
        stopStateDis();
        flag = false;
    end
end

if flag
    %% Wait for correct handshake ready signal as the MCU should keep sending ReadyFlag 
    outPutStatus(TempApp,'Wait for Handshake...');
    pause(5/1000);
    Transtate = char(readline(McuSerial));
    flush(McuSerial,"input");
    pause(5/1000);
    tic
    while ~strcmp(Transtate,ExoP.ReadyFlag)
        Transtate = char(readline(McuSerial));
        TempApp.txtInfo.Value = Transtate;
        if(toc > 30)
            ExoP.stopFlag = 3;
            delete(McuSerial);
            clear McuSerial;
            stopStateDis();
            flag = false;
        end
    end
end

if flag
    %% Notice that the initial command send to low-level controller during
    % handshake process indicates that mode = 1 and side = 0
    TransState = "TL0000TR0000M10";
    writeline(McuSerial,TransState);  % send the data
    outPutStatus(TempApp,'Sucessful Handshake.');
    TempApp.txtMode.Value = ['Last State: Exit',10,'Curr State: Exit',10,'Cycels: 0'];
    pause(5/1000);
end

end