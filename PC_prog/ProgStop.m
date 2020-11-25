function ProgStop(Ttimer,~)
% Stop the usart communication and timer
global P;

%% Delete the timer
delete(Ttimer);
clear Ttimer
disp('Timer stopped!');

%% Send command for stop status to low-level controller
McuPort = P.config{1,1};
TransState = ['TL0000TR0000M00',char(13)];
flushoutput(McuPort);      % flush the output buffer
fprintf(McuPort,TransState);    % send the data
% wait until data are all sent
while McuPort.BytesToOutput ~= 0  
end

%% Stop the serial port 
%stopasync(McuPort);
fclose(McuPort);
delete(McuPort);
disp('Serial port shut down!')

%% Saving data
DataSaving();


end