function Timer_Init(Main_Freq)
global P;
% create and set timer properties
% 1/Main_Freq should be larger than 0.001
Timer1 = timer('BusyMode','drop','ExecutionMode','fixedRate','Period',1/Main_Freq,'TimerFcn',@TimerCallback);  % remind the main frequency here
%Timer1.TimerFcn = {@TimerCallback,P};  % set the callback of the timer
P.config{2,1} = Timer1;  % store the configuration of timer1
tic
start(Timer1);       % start the timer 
end