function TimerStop(Ttimer,~)
% Stop the usart communication and timer
global ExoP;
%% Delete the timer
delete(Ttimer);
clear Ttimer
% Check if serial port error or not
if isvalid(ExoP.config{1,1})
    ProgStop();
end
%% Saving data
stopStateDis();
DataSaving();
end