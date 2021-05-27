function DataSaving()
% Saving experimental data for further data analysis
global ExoP;
global TempApp;
% %% Saving program time information
% assignin('base','TimeAll',ExoP.TimeAll);
% assignin('base','TimeTrans',ExoP.TransTime);
% %% Saving data from MCU
% assignin('base','torqueTL',ExoP.torqueTL);
% assignin('base','forceLL',ExoP.forceLL);
% assignin('base','angleAL',ExoP.angleAL);
% assignin('base','torqueTR',ExoP.torqueTR);
% assignin('base','forceLR',ExoP.forceLR);
% assignin('base','angleAR',ExoP.angleAR);
% assignin('base','angleP',ExoP.angleP);
% assignin('base','angleY',ExoP.angleY);
% assignin('base','adotPV',ExoP.adotPV);
% 
% %% Saving data used by control
% assignin('base','AlphaMean',ExoP.AlphaMean);   % from ExoP.angleP
% assignin('base','AlphaDot',ExoP.AlphaDot);     % calculated based on ExoP.AlphaMean and ExoP.TimeAll
% assignin('base','BetaMean',ExoP.BetaMean);     % from ExoP.angleY
% % more ...
% 
% %% Saving data from PC
% assignin('base','DesiredTorque',ExoP.DesiredTorque);
% assignin('base','MotionMode',ExoP.MotionMode);

%% Saving P for experimental configuration parameters
assignin('base','ExoP',ExoP);
outPutStatus(TempApp,'Data Saved.');
% fileName = input('File name:\n','s');
% fileName = datetime('now');
% fileName = datestr(fileName,'yyyymmddTHHMMSS');
% save(fileName);

end