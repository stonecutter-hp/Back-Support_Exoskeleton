function testTimerStop(Ttimer,~)
% Stop the usart communication and timer
global testP;
%% Delete the timer
delete(Ttimer);
clear Ttimer
% disp('Timer stopped!');
% Check if serial port error or not

%% Saving data
testDataSaving();
teststopStateDis();
end