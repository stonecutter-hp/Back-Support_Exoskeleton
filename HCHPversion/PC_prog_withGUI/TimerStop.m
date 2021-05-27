function TimerStop(Ttimer,~)
% Stop the usart communication and timer
global ExoP;
%% Delete the timer
delete(Ttimer);
clear Ttimer
% disp('Timer stopped!');
% Check if serial port error or not
if isvalid(ExoP.config{1,1})
    ProgStop();
end
%% Saving data
DataSaving();
stopStateDis();
end