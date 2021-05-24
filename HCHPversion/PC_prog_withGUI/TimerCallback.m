function TimerCallback(Ttimer,~)
global ExoP;
global TempApp;
% need to consider about the time data saving
% Update Flag: 1: Enable update;  0: Waiting for permission of update  


%% Store each callback time, first "tic" is started in Timer_Init
ExoP.TimeAll = [ExoP.TimeAll; toc];

%% Receive --> Control --> Send
% %*********************** Receive ***********************
% [Control_Update,Send_Update] = Receive_McuData();
% 
% %*********************** Control ***********************
% if Control_Update == 1
%     Control();
% end
% %************************ Send *************************
% if Send_Update == 1
%     Send_Data();
% end

% % % % % % %% For test only
% % % % % % McuSerial = P.config{1,1};
% % % % % % TransState = 'TL0000TR0000M00';
% % % % % % flushoutput(McuSerial);      % flush the output buffer
% % % % % % fprintf(McuSerial,TransState);    % send the data
% % % % % % % wait until data are all sent
% % % % % % while McuSerial.BytesToOutput ~= 0  
% % % % % % end

%% If the running time overlarger than preset running time, then stop program
% % Potential stop condition for practical application: Make sure the main
% % program only stop when info package feedback turn to resume to a safe
% % operation point  
% if (ExoP.TimeAll(end) > ExoP.MaxRunTime && (ExoP.MotionMode(end,1) == 1 || ExoP.MotionMode(end,1) == 0))
%     stop(Ttimer);
% end

% Program stop condition for testing
if (ExoP.TimeAll(end) > ExoP.MaxRunTime)
    stop(Ttimer);
    outPutStatus(TempApp,'Program Auto Stopped.');
end


end



