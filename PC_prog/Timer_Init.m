function Timer_Init(Main_Freq)
global P;
% create and set timer properties
% 1/Main_Freq should be larger than 0.001s which means the loop should run
% at frequncy of less than 1kHz
% remind the main frequency here
Timer1 = timer('BusyMode','drop','ExecutionMode','fixedRate','Period',1/Main_Freq,'TimerFcn',@TimerCallback);  
% Timer1.TimerFcn = {@TimerCallback,P};  % set the callback of the timer
Timer1.StopFcn = @ProgStop;
Timer1.StartFcn = @ProgStart;
P.config{2,1} = Timer1;  % store the configuration of timer1
start(Timer1);       % start the timer 


end