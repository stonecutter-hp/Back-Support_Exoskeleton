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
end
%************************ Send *************************
if (Send_Update == 1)
    Send_Data();
end

% % % % % % %% For test only
% % % % % % McuSerial = P.config{1,1};
% % % % % % TransState = 'TL0000TR0000M00';
% % % % % % flushoutput(McuSerial);      % flush the output buffer
% % % % % % fprintf(McuSerial,TransState);    % send the data
% % % % % % % wait until data are all sent
% % % % % % while McuSerial.BytesToOutput ~= 0  
% % % % % % end

%% If the running time overlarger than preset running time, then stop program
if (P.TransTime(end) > P.MaxRunTime)
    stop(Ttimer);
end


end



