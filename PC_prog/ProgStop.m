function ProgStop()
% Stop the usart communication and timer
global P;
%% Stop the timer
TimerExist = timerfind;
stop(TimerExist);
delete(TimerExist);
clear TimerExist
disp('Timer stopped!');

%% Stop the serial port 
McuPort = P.config{1,1};
%stopasync(McuPort);
fclose(McuPort);
delete(McuPort);
disp('Serial port shut down!')



end