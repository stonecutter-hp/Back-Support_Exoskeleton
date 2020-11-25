function DataSaving()
% Saving experimental data for further data analysis
global P;
%% Saving program time information
assignin('base','TimeAll',P.TimeAll);

%% Saving data from MCU
assignin('base','angleAL',P.angleAL);
assignin('base','forceLL',P.forceLL);
assignin('base','angleTL',P.angleTL);
assignin('base','angleAR',P.angleAR);
assignin('base','forceLR',P.forceLR);
assignin('base','angleTR',P.angleTR);
assignin('base','angleP',P.angleP);
assignin('base','angleY',P.angleY);
assignin('base','adotPV',P.adotPV);

%% Saving data used by control
assignin('base','AngleMean',P.AngleMean);
assignin('base','AlphaDot',P.AlphaDot);
assignin('base','BetaMean',P.BetaMean);
% more ...

%% Saving data from PC
assignin('base','DesiredTorque',P.DesiredTorque);
assignin('base','MotionMode',P.MotionMode);

%% Saving P for experimental configuration parameters
assignin('base','P',P);

% fileName = input('File name:\n','s');
% fileName = datetime('now');
% fileName = datestr(fileName,'yyyymmddTHHMMSS');
% save(fileName);

end