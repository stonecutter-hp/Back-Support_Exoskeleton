function Timer_Init(Main_Freq)
% In this Timer_Init fcn, the processing logic is:
% Timer setting and initialization
% --> Serial port initialization: -> Handshake with low-level controller
% --> Start timer loop
global ExoP;
global TempApp;
% create and set timer properties
% 1/Main_Freq should be larger than 0.001s which means the loop should run
% at frequncy of less than 1kHz
% remind the main frequency here
Timer1 = timer('BusyMode','drop','ExecutionMode','fixedRate',...
               'Period',1/Main_Freq,'TimerFcn',@TimerCallback);  
% Timer1 = timer('BusyMode','drop','ExecutionMode','fixedRate',...
%                'Period',1/Main_Freq,'TimerFcn',@TimerCallback);
% Timer1.TimerFcn = {@TimerCallback,P};  % set the callback of the timer
Timer1.StartFcn = @ProgStart;
Timer1.StopFcn = @TimerStop;
Timer1.ErrorFcn = @TimerError;
ExoP.config{2,1} = Timer1;  % store the configuration of timer1
outPutStatus(TempApp,'High-level Control Running.');
pause(5/1000);
start(Timer1);       % start the timer 
end