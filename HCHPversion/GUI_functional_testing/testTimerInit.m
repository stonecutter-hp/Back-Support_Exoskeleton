function testTimerInit(Main_Freq)
global testP;
global testTempApp;

testTimer = timer('BusyMode','drop','ExecutionMode','fixedRate',...
               'Period',1/Main_Freq,'TimerFcn',@testTimerCallback);
testTimer.StartFcn = @testProgStart;
testTimer.StopFcn = @testTimerStop;
testP.config{2,1} = testTimer;  % store the configuration of test timer
outPutStatus(testTempApp,'Timer Running');
pause(5/1000);
start(testTimer);       % start the timer
end