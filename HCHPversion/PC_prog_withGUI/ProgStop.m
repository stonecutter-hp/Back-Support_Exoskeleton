function ProgStop(Ttimer,~)
% Stop the usart communication and timer
global ExoP;
global TempApp;
%% Delete the timer
delete(Ttimer);
clear Ttimer
% disp('Timer stopped!');

%% Send command for stop status to low-level controller
% McuSerial = ExoP.config{1,1};
% TransState = 'TL0000TR0000M00';
% flushoutput(McuSerial);           % flush the output buffer
% fprintf(McuSerial,TransState);    % send the data
% % wait until data are all sent
% while McuSerial.BytesToOutput ~= 0  
% end

%% Stop the serial port 
%stopasync(McuPort);
% fclose(McuSerial);
% delete(McuSerial);
% disp('Serial port shut down!')
outPutStatus(TempApp,'Serial Port Shut Down.');
%% Saving data
DataSaving();
outPutStatus(TempApp,'Data Saved.');

end