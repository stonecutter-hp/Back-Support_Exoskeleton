function testMain()
% The logic of this function:
% Set a timer according to input frequncy and running time (Corresponds to
% timer initialization) --> Run the timer callback to add the global
% indicator (Corresponding to the receiving + control + send) --> Print out
% the indicator (Corresponds to the info printing) --> Program stop
% (Corresponds to to the stop) 

global testP;
MainFreq = testP.MainFreq;          % Set program frequency
testTimerInit(MainFreq);


end