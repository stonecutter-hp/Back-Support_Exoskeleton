function DataSaving()
% Saving experimental data for further data analysis
global P;
%% Saving program time information
assignin('base','TimeAll',P.TimeAll);

%% Saving data from MCU
assignin('base','angleAL',P.angleAL);
assignin('base','angleSL',P.angleSL);
assignin('base','angleR',P.angleR);
assignin('base','angleP',P.angleP);
assignin('base','angleY',P.angleY);


%% Saving data used by control
% assignin('base','AngleMean',P.AngleMean);
% more ...

%% Saving data from PC
% assignin('base','MotionMode',P.MotionMode);
% assignin('base','DesiredTorque',P.DesiredTorque);

%% Saving P for experimental configuration parameters
assignin('base','P',P);

% fileName = input('File name:\n','s');
% fileName = datetime('now');
% fileName = datestr(fileName,'yyyymmddTHHMMSS');
% save(fileName);

end