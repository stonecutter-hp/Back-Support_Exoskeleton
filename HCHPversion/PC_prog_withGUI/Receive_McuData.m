function [Control_Update,Send_Update] = Receive_McuData()
% This program is to receive and process the info package from low-level
% controller
global ExoP;
global TempApp;
%% Read data from MCU
McuSerial = ExoP.config{1,1};
TransState = readline(McuSerial);   % read the input buffer
TransState = char(TransState);
% ensure the data completeness
while numel(TransState) ~= ExoP.ReceiveDataNum-2
    TransState = readline(McuSerial);   % read the input buffer
    TransState = char(TransState);
    pause(2/1000);
end
if McuSerial.NumBytesAvailable >= 3*(ExoP.ReceiveDataNum-2)
    flush(McuSerial,"input");
end

%% Decompose the data from MCU from characters to numbers
% Here the specific form for recieved data is designed as: 
% MxTLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxxcLxxxxcRxxxxvLxxxxxvRxxxxx\r\n
% TL/Rxxxx: (Nm) Torque feedback for left/right transmission system 
% LL/Rxxxx: (N) Load cell feedback for cable force of left/right transmission system
% AL/Rxxxxx: (deg) Potentiometer feedback for hip angle feedback
%                  first number indicate sign: 0 for -, 1 for +
% Pxxxxx: (deg) Pitch angle for trunk
% Yxxxxx: (deg) yaw angle for trunk
%               first number indicate sign: 0 for -, 1 for + 
% Vxxxxx: (deg/s) Pitch angular velocity for trunk
%                 first number indicate sign: 0 for -, 1 for +
% CL/Rxxxx: PWM Cycle Duty
%           first number indicate sign: 0 for -(loosening cable), 1 for +(tighting cable)
% cL/Rxxxx: (A) Motor Current Feedback from Driver
% vL/Rxxxxx: (rpm) Motor Velocity Feedback from Driver
% Mx: Marking flag to show if the MCU data is real-time with successful receiving last command from PC
% Notice: The last two are terminator for PC receiveing '\r\n'

position = 3;
flag = true;
% Also some security operation can be added if over P.MaxDelay loop no 
% command is recieved from PC like: stop(P.config{2,1});

if(TransState(position-2) == 'M')
    % Store this time's delay flag
    ExoP.DelayMark = [ExoP.DelayMark; TransState(position-1)];
    if(~(ExoP.DelayEnable) && ExoP.DelayNumber < ExoP.MaxDelay)
        if(ExoP.DelayMark(end) ~= ExoP.SwitchFlag)
            Control_Update = 1;
            Send_Update = 1;
            ExoP.SwitchFlag = ExoP.DelayMark(end);
            ExoP.DelayNumber = 0;
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
            ExoP.DelayNumber = ExoP.DelayNumber+1;
        end
    else
        Control_Update = 1;
        Send_Update = 1;
        ExoP.DelayNumber = 0;
    end
else
    % False recieve cycle
    Control_Update = 0;
    Send_Update = 0; 
    flag = false;
end

% if 
if(Control_Update && flag)
    % TLxxxx
    if(ExoP.RecItem(1) == 1 && flag)
        if(TransState(position) == 'T' && TransState(position+1) == 'L')
            TransTorqueTL = (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                            (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
        position = position+6;
    else
        TransTorqueTL = 0;
    end
    % LLxxxx
    if(ExoP.RecItem(2) == 1 && flag)
        if(TransState(position) == 'L' && TransState(position+1) == 'L')
            TransForceLL = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                           (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
        position = position+6;
    else
        TransForceLL = 0;
    end
    % ALxxxxx
    if(ExoP.RecItem(3) == 1 && flag)
        if(TransState(position) == 'A' && TransState(position+1) == 'L')
            TransAngleAL = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                           (TransState(position+5)-48)*1+(TransState(position+6)-48)*0.1;
            if(TransState(position+2) == '0')
                TransAngleAL = -1*TransAngleAL;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;            
        end
        position = position+7;
    else
        TransAngleAL = 0;
    end
    % TRxxxx
    if(ExoP.RecItem(4) == 1 && flag)
        if(TransState(position) == 'T' && TransState(position+1) == 'R')
            TransTorqueTR = (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                            (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false; 
        end
        position = position+6;
    else
        TransTorqueTR = 0;
    end
    % LRxxxx
    if(ExoP.RecItem(5) == 1 && flag)
        if(TransState(position) == 'L' && TransState(position+1) == 'R')
            TransForceLR = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                           (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;            
        end
        position = position+6;
    else
        TransForceLR = 0;
    end
    % ARxxxxx
    if(ExoP.RecItem(6) == 1 && flag)
        if(TransState(position) == 'A' && TransState(position+1) == 'R')
            TransAngleAR = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                           (TransState(position+5)-48)*1+(TransState(position+6)-48)*0.1;
            if(TransState(position+2) == '0')
                TransAngleAR = -1*TransAngleAR;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;            
        end
        position = position+7;
    else
        TransAngleAR = 0;
    end
    % Pxxxxx
    if(ExoP.RecItem(7) == 1 && flag)
        if(TransState(position) == 'P')
            TransAngleP = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                          (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
            if(TransState(position+1) == '0')
                TransAngleP = -1*TransAngleP;
            end    
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;             
        end
        position = position+6;
    else
        TransAngleP = 0;
    end
    % Yxxxxx
    if(ExoP.RecItem(8) == 1 && flag)
        if(TransState(position) == 'Y')
            TransAngleY = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                          (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
            if(TransState(position+1) == '0')
                TransAngleY = -1*TransAngleY;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;             
        end
        position = position+6;
    else
        TransAngleY = 0;
    end
    %Vxxxxx
    if(ExoP.RecItem(9) == 1 && flag)
        if(TransState(position) == 'V')
            TransVeloV = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                         (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
            if(TransState(position+1) == '0')
                TransVeloV = -1*TransVeloV;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;              
        end
        position = position+6;
    else
        TransVeloV = 0;
    end
    %CLxxxx
    if(ExoP.RecItem(10) == 1 && flag)
        if(TransState(position) == 'C' && TransState(position+1) == 'L')
            TransPWML = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                        (TransState(position+5)-48)*1;
            if(TransState(position+2) == '0')
                TransPWML = -1*TransPWML;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
        position = position+6;
    else
        TransPWML = 0;
    end
    %CRxxxx
    if(ExoP.RecItem(11) == 1 && flag)
        if(TransState(position) == 'C' && TransState(position+1) == 'R')
            TransPWMR = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                        (TransState(position+5)-48)*1;
            if(TransState(position+2) == '0')
                TransPWMR = -1*TransPWMR;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
        position = position+6;
    else
        TransPWMR = 0;
    end
    %cLxxxx
    if(ExoP.RecItem(12) == 1 && flag)
        if(TransState(position) == 'c' && TransState(position+1) == 'L')
            TransCurrL = (TransState(position+3)-48)*1+(TransState(position+4)-48)*0.1+...
                        (TransState(position+5)-48)*0.01;
            if(TransState(position+2) == '0')
                TransCurrL = -1*TransCurrL;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
        position = position+6;
    else
        TransCurrL = 0;
    end
    %cRxxxx
    if(ExoP.RecItem(13) == 1 && flag)
        if(TransState(position) == 'c' && TransState(position+1) == 'R')
            TransCurrR = (TransState(position+3)-48)*1+(TransState(position+4)-48)*0.1+...
                        (TransState(position+5)-48)*0.01;
            if(TransState(position+2) == '0')
                TransCurrR = -1*TransCurrR;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
        position = position+6;
    else
        TransCurrR = 0;
    end
    %vLxxxxx
    if(ExoP.RecItem(14) == 1 && flag)
        if(TransState(position) == 'v' && TransState(position+1) == 'L')
            TransMotorVelL = (TransState(position+3)-48)*1000+(TransState(position+4)-48)*100+...
                             (TransState(position+5)-48)*10+(TransState(position+6)-48)*1;
            if(TransState(position+2) == '0')
                TransMotorVelL = -1*TransMotorVelL;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
        position = position+7;
    else
        TransMotorVelL = 0;
    end  
    %vRxxxxx
    if(ExoP.RecItem(15) == 1 && flag)
        if(TransState(position) == 'v' && TransState(position+1) == 'R')
            TransMotorVelR = (TransState(position+3)-48)*1000+(TransState(position+4)-48)*100+...
                             (TransState(position+5)-48)*10+(TransState(position+6)-48)*1;
            if(TransState(position+2) == '0')
                TransMotorVelR = -1*TransMotorVelR;
            end
        else
            Control_Update = 0;
            Send_Update = 0;
            flag = false;
        end
    else
        TransMotorVelR = 0;
    end
    
end

if flag
    ExoP.TransTime = [ExoP.TransTime; toc];
    ExoP.torqueTL = [ExoP.torqueTL; TransTorqueTL];
    ExoP.forceLL = [ExoP.forceLL; TransForceLL];
    ExoP.angleAL = [ExoP.angleAL; TransAngleAL];
    ExoP.torqueTR = [ExoP.torqueTR; TransTorqueTR];
    ExoP.forceLR = [ExoP.forceLR; TransForceLR];
    ExoP.angleAR = [ExoP.angleAR; TransAngleAR];
    ExoP.angleP = [ExoP.angleP; TransAngleP];
    ExoP.angleY = [ExoP.angleY; TransAngleY];
    ExoP.adotPV = [ExoP.adotPV; TransVeloV];
    ExoP.PWM_L = [ExoP.PWM_L; TransPWML];
    ExoP.PWM_R = [ExoP.PWM_R; TransPWMR];
    ExoP.CurrL = [ExoP.CurrL; TransCurrL];       % (A) left motor current
    ExoP.CurrR = [ExoP.CurrR; TransCurrR];       % (A) right motor current
    ExoP.MotorVelL = [ExoP.MotorVelL; TransMotorVelL];   % (rpm) left motor velocity
    ExoP.MotorVelR = [ExoP.MotorVelR; TransMotorVelR];   % (rpm) right motor velocity
    % JUST NEEDED FOR SOME STRATEGY: Other info from calculaion
    if size(ExoP.adotPV,1) == 1 % Fisrt cycle
        Trans_tdotTL = 0;
        Trans_tdotTR = 0;
        Trans_adotAL = ExoP.adotPV; 
        Trans_adotAR = ExoP.adotPV;
    else
        % Calculation
        delta_t = ExoP.TransTime(end)-ExoP.TransTime(end-1);
        Trans_tdotTL = (ExoP.torqueTL(end)-ExoP.torqueTL(end-1))/delta_t;
        Trans_tdotTR = (ExoP.torqueTR(end)-ExoP.torqueTR(end-1))/delta_t;
        Trans_adotAL = (ExoP.angleAL(end)-ExoP.angleAL(end-1))/delta_t;
        Trans_adotAR = (ExoP.angleAR(end)-ExoP.angleAR(end-1))/delta_t;
    end
    ExoP.tdotTL = [ExoP.tdotTL;Trans_tdotTL];
    ExoP.tdotTR = [ExoP.tdotTR;Trans_tdotTR];
    ExoP.adotAL = [ExoP.adotAL;Trans_adotAL];
    ExoP.adotAR = [ExoP.adotAR;Trans_adotAR];
    
    % TransState
    TempApp.txtInfo.Value = TransState;
end

%%%%%%%%%%%%%%% Here can add more data decompostition processing %%%%%%%%%%%%%

end