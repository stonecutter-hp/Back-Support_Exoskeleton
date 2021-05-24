function TimerStop(Ttimer,~)
% Stop the usart communication and timer
%% Delete the timer
delete(Ttimer);
clear Ttimer
% disp('Timer stopped!');

end