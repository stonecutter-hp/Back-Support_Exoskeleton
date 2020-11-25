function TimerCallback(Ttimer,~)
global P;
% need to consider about the time data saving
% Update Flag: 1: Enable update;  0: Waiting for permission of update
Receive_Update = 1;    


%% Store each callback time, first "tic" is started in Timer_Init
P.TransTime = P.TransTime+ toc;  % Record start time of each cycle
tic
P.TimeAll = [P.TimeAll P.TransTime];

%% Receive --> Control --> Send
%*********************** Receive ***********************
if (Receive_Update == 1)
    [Control_Update,Send_Update] = Receive_McuData();
    Receive_Update = 0;
end
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



