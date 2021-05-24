function ProgStop()
% Program Stop Processing Function
global ExoP;
global TempApp;
%% Send command for stop status to low-level controller
McuSerial = ExoP.config{1,1};
TransState = 'TL0000TR0000M10';
flushoutput(McuSerial);           % flush the output buffer
fprintf(McuSerial,TransState);    % send the data
% wait until data are all sent
while McuSerial.BytesToOutput ~= 0  
end

%% Stop the serial port 
% stopasync(McuPort);
fclose(McuSerial);
delete(McuSerial);
% disp('Serial port shut down!')
outPutStatus(TempApp,'Serial Port Shut Down.');
pause(5/1000);
%% Saving data
DataSaving();
outPutStatus(TempApp,'Data Saved.');
set(TempApp.btnRun,'value',0);

end