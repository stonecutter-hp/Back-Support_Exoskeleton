function TimerCallback(Ttimer,~)
global ExoP;
global TempApp;
% need to consider about the time data saving
% Update Flag: 1: Enable update;  0: Waiting for permission of update  


%% Store each callback time, first "tic" is started in Timer_Init
ExoP.TimeAll = [ExoP.TimeAll; toc];

%% Receive --> Control --> Send
%*********************** Receive ***********************
[Control_Update,Send_Update] = Receive_McuData();

%*********************** Control ***********************
if Control_Update == 1
    Control();
    % If UID and RTG are all set for testing mode
    if ExoP.UIDStrategy == 0 && ExoP.UIDStrategy == 0 
        TempApp.txtMode.Value = 'TESTING';  % FOR INDICATION ONLY
    else
        highLevelCommandDis();
    end
end
%************************ Send *************************
if Send_Update == 1
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

%% Timer loop stop condition
% Stop condition for practical application (Before complete high-level UID
% strategy): 
% Running time has exceed the expected running time, meanwhile the current
% motion state is Exit or Standing 
if (ExoP.TimeAll(end) > ExoP.MaxRunTime ... 
    && (ExoP.MotionMode(end,2) == ExoP.Exit ... 
    || ExoP.MotionMode(end,2) == ExoP.Standing))
    stop(Ttimer);
end

% % Stop condition for practical application (After complete high-level UID
% % strategy): 
% % The bending cycle has exceed the expected bending cycles, meanwhile the
% % current motion state is Exit or Standing 
% if (ExoP.BendCycle > ExoP.MaxBendCycles ... 
%     && (ExoP.MotionMode(end,2) == ExoP.Exit ... 
%     || ExoP.MotionMode(end,2) == ExoP.Standing))
%     stop(Ttimer);
% end



end



