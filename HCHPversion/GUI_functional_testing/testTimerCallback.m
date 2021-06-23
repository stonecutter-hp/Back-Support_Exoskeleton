function testTimerCallback(Ttimer,~)
global testP;
global testTempApp;

    
testP.runIndicator = abs(testP.runIndicator)+testP.alogMode;
if testP.negEnable == 1
    testP.runIndicator = -1*testP.runIndicator;
end
testP.data = [testP.data,testP.runIndicator];
testTempApp.txtIndicator.Value = num2str(testP.runIndicator);


if (toc > testP.MaxRunTime)
    stop(Ttimer);
end

end