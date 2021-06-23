function testDataSaving()
global testP;
global testTempApp;
%% Saving P for experimental configuration parameters
assignin('base','testP',testP);
% fileName = input('File name:\n','s');
fileName = datetime('now');
fileName = datestr(fileName,'yyyymmddTHHMMSS');
[file,path] = uiputfile('*.mat','Workspace File',fileName);
if isequal(file,0) || isequal(path,0)
    outPutStatus(testTempApp,'Cancel Saving Data');
else
    save([path,file],'testP');
    outPutStatus(testTempApp,'Data Saved.');
end
end