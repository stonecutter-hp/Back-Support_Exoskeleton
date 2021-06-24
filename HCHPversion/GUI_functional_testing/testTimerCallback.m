function testTimerCallback(Ttimer,~)
global testP;
global testTempApp;

testP.TransTime = [testP.TransTime,toc];
%% Corresponds to receiving data
testP.runIndicator = abs(testP.runIndicator)+testP.alogMode;
if testP.negEnable == 1
    testP.runIndicator = -1*testP.runIndicator;
end
testP.data = [testP.data,testP.runIndicator];
testTempApp.txtIndicator.Value = num2str(testP.runIndicator);

%% Corresponds to UID strategy
if testP.VCClickFlag
    switch testP.VCStatus
        case {0,1}
            testP.VCStatus = testP.VCStatus+1;
        case 2
            testP.VCStatus = testP.VCStatus-2;
        otherwise
            testP.VCStatus = 0;
    end
    testTempApp.txtTestWearStatus.Value = num2str(testP.VCStatus);
    mode = 1;
    testP.VCClickFlag = 0;
else
    mode = 0;
end

testP.Status = [testP.Status,mode];


if (testP.TransTime(end) > testP.MaxRunTime)
    stop(Ttimer);
end

end