function ProgStop(Ttimer,~)
% Stop the usart communication and timer
global P;

%% Delete the timer
delete(Ttimer);
clear Ttimer
disp('Timer stopped!');

%% Send command for stop status to low-level controller
McuSerial = P.config{1,1};
TransState = 'TL0000TR0000M00';
flushoutput(McuSerial);           % flush the output buffer
fprintf(McuSerial,TransState);    % send the data
% wait until data are all sent
while McuSerial.BytesToOutput ~= 0  
end

%% Stop the serial port 
%stopasync(McuPort);
fclose(McuSerial);
delete(McuSerial);
disp('Serial port shut down!')

%% Saving data
DataSaving();


end