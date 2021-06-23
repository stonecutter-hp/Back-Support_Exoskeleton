function testDataSaving()
global testP;
global testTempApp;
%% Saving P for experimental configuration parameters
assignin('base','testP',testP);
outPutStatus(testTempApp,'Data Saved.');

end