function TimerCallback(Ttimer,~)
global P;
% need to consider about the time data saving
% Update Flag: 1: Enable update;  0: Waiting for permission of update  


%% Store each callback time, first "tic" is started in Timer_Init
P.TimeAll = [P.TimeAll toc];

%% Receive --> Control --> Send
%*********************** Receive ***********************
[Control_Update,Send_Update] = Receive_McuData();

%*********************** Control ***********************
if (Control_Update == 1)
    Control();
    Control_Update = 0;
end
%************************ Send *************************
if (Send_Update == 1)
    Send_Data();
    Send_Update = 0;
end
%% If the running time overlarger than preset running time, then stop program
if (P.TransTime > P.MaxRunTime)
    stop(Ttimer);
end


end



