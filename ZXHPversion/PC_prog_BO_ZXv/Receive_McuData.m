function [Control_Update,Send_Update] = Receive_McuData()
% This program is to receive and process the info package from low-level
% controller
global ExoP;
global TempApp;
%% Read data from MCU
McuSerial = ExoP.config{1,1};
TransStateTemp = readline(McuSerial);   % read the input buffer
TransState = char(TransStateTemp);
% ensure the data completeness
while numel(TransState) ~= ExoP.ReceiveDataNum-2
    TransStateTemp = readline(McuSerial);   % read the input buffer
    TransState = char(TransStateTemp);
%     pause(2/1000);
end
if McuSerial.NumBytesAvailable >= 3*(ExoP.ReceiveDataNum-2)
    flush(McuSerial,"input");
end

%% Decompose the data from MCU from characters to numbers
% /**
%  * @ MCU to PC protocol: MxTLxxxxxALxxxxxVLxxxxxHLxxxxxTRxxxxxARxxxxxVRxxxxxHRxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxxDLxxxxDRxxxxSxxx\r\n
%  * TL/Rxxxxx: (Nm) Torque feedback for left/right transmission system 
%  *                 first number indicate sign: 0 for -, 1 for +
%  * AL/Rxxxxx: (deg) Hip angle feedback from potentiometer
%  *                  first number indicate sign: 0 for -, 1 for +
%  * VL/Rxxxxx: (deg/s) Hip angular velocity feedback
%  *                    first number indicate sign: 0 for -, 1 for +
%  * HL/Rxxxxx: (deg/s^2) Hip angular acceleration feedback
%  *                      first number indicate sign: 0 for -, 1 for +
%  * Pxxxxx: (deg) Pitch angle for trunk
%  *               first number indicate sign: 0 for -, 1 for +
%  * Yxxxxx: (deg) yaw angle for trunk
%  *               first number indicate sign: 0 for -, 1 for + 
%  * Vxxxxx: (deg/s) Pitch angular velocity for trunk
%  *                 first number indicate sign: 0 for -, 1 for +
%  * CL/Rxxxx: PWM Cycle Duty
%  *           first number indicate sign: 0 for -(loosening cable), 1 for +(tighting cable)
%  * DL/Rxxxx: Desired torque command
%  * Sxxx    : User motion status indicator
%  *   First number coresponds to MotionType
%  *   Second number coresponds to AsymSide
%  *   Third number corresponds to BendTech 
%  * Mx: Marking flag to show if the MCU data is real-time with successful receiving last command from PC
%  * Notice: The last two are terminator for PC receiveing '\r\n'
%  */

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
    % TLxxxxx
    if(ExoP.RecItem(1) == 1)
        if(TransState(position) == 'T' && TransState(position+1) == 'L')
            TransTorqueTL = (TransState(position+3)-48)*10+(TransState(position+4)-48)*1+...
                            (TransState(position+5)-48)*0.1+(TransState(position+6)-48)*0.01;
            if(TransState(position+2) == '0')
                TransTorqueTL = -1*TransTorqueTL;
            end
        else
            flag = false;
        end
        position = position+7;
    else
        TransTorqueTL = 0;
    end  
    
    % ALxxxxx
    if(ExoP.RecItem(2) == 1 && flag)
        if(TransState(position) == 'A' && TransState(position+1) == 'L')
            TransAngleAL = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                            (TransState(position+5)-48)*1+(TransState(position+6)-48)*0.1;
            if(TransState(position+2) == '0')
                TransAngleAL = -1*TransAngleAL;
            end
        else
            flag = false;
        end
        position = position+7;
    else
        TransAngleAL = 0;
    end      
    
    % VLxxxxx
    if(ExoP.RecItem(3) == 1 && flag)
        if(TransState(position) == 'V' && TransState(position+1) == 'L')
            TransAngleVL = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                            (TransState(position+5)-48)*1+(TransState(position+6)-48)*0.1;
            if(TransState(position+2) == '0')
                TransAngleVL = -1*TransAngleVL;
            end
        else
            flag = false;
        end
        position = position+7;
    else
        TransAngleVL = 0;
    end          
    
    % HLxxxxx
    if(ExoP.RecItem(4) == 1 && flag)
        if(TransState(position) == 'H' && TransState(position+1) == 'L')
            TransAngleACCL = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                            (TransState(position+5)-48)*1+(TransState(position+6)-48)*0.1;
            if(TransState(position+2) == '0')
                TransAngleACCL = -1*TransAngleACCL;
            end
        else
            flag = false;
        end
        position = position+7;
    else
        TransAngleACCL = 0;
    end 

    % TRxxxxx
    if(ExoP.RecItem(5) == 1 && flag)
        if(TransState(position) == 'T' && TransState(position+1) == 'R')
            TransTorqueTR = (TransState(position+3)-48)*10+(TransState(position+4)-48)*1+...
                            (TransState(position+5)-48)*0.1+(TransState(position+6)-48)*0.01;
            if(TransState(position+2) == '0')
                TransTorqueTR = -1*TransTorqueTR;
            end
        else
            flag = false;
        end
        position = position+7;
    else
        TransTorqueTR = 0;
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
            flag = false;
        end
        position = position+7;
    else
        TransAngleAR = 0;
    end      
    
    % VRxxxxx
    if(ExoP.RecItem(7) == 1 && flag)
        if(TransState(position) == 'V' && TransState(position+1) == 'R')
            TransAngleVR = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                            (TransState(position+5)-48)*1+(TransState(position+6)-48)*0.1;
            if(TransState(position+2) == '0')
                TransAngleVR = -1*TransAngleVR;
            end
        else
            flag = false;
        end
        position = position+7;
    else
        TransAngleVR = 0;
    end          
    
    % HRxxxxx
    if(ExoP.RecItem(8) == 1 && flag)
        if(TransState(position) == 'H' && TransState(position+1) == 'R')
            TransAngleACCR = (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                            (TransState(position+5)-48)*1+(TransState(position+6)-48)*0.1;
            if(TransState(position+2) == '0')
                TransAngleACCR = -1*TransAngleACCR;
            end
        else
            flag = false;
        end
        position = position+7;
    else
        TransAngleACCR = 0;
    end     

    % Pxxxxx
    if(ExoP.RecItem(9) == 1 && flag)
        if(TransState(position) == 'P')
            TransAngleP = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                          (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
            if(TransState(position+1) == '0')
                TransAngleP = -1*TransAngleP;
            end    
        else
            flag = false;
        end
        position = position+6;
    else
        TransAngleP = 0;
    end    
    
    % Yxxxxx
    if(ExoP.RecItem(10) == 1 && flag)
        if(TransState(position) == 'Y')
            TransAngleY = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                          (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
            if(TransState(position+1) == '0')
                TransAngleY = -1*TransAngleY;
            end
        else
            flag = false;
        end
        position = position+6;
    else
        TransAngleY = 0;
    end
    
    %Vxxxxx
    if(ExoP.RecItem(11) == 1 && flag)
        if(TransState(position) == 'V')
            TransVeloV = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                         (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
            if(TransState(position+1) == '0')
                TransVeloV = -1*TransVeloV;
            end
        else
            flag = false;
        end
        position = position+6;
    else
        TransVeloV = 0;
    end
    
    %CLxxxx
    if(ExoP.RecItem(12) == 1 && flag)
        if(TransState(position) == 'C' && TransState(position+1) == 'L')
            TransCL= (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                     (TransState(position+5)-48)*1;
            if(TransState(position+2) == '0')
                TransCL = -1*TransCL;
            end
        else
            flag = false;
        end
        position = position+6;
    else
        TransCL = 0;
    end
    
    %CRxxxx
    if(ExoP.RecItem(13) == 1 && flag)
        if(TransState(position) == 'C' && TransState(position+1) == 'R')
            TransCR= (TransState(position+3)-48)*100+(TransState(position+4)-48)*10+...
                     (TransState(position+5)-48)*1;
            if(TransState(position+2) == '0')
                TransCR = -1*TransCR;
            end
        else
            flag = false;
        end
        position = position+6;
    else
        TransCR = 0;
    end
    
    %DLxxxx
    if(ExoP.RecItem(14) == 1 && flag)
        if(TransState(position) == 'D' && TransState(position+1) == 'L')
            TransDL= (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                     (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        else
            flag = false;
        end
        position = position+6;
    else
        TransDL = 0;
    end

    %DRxxxx
    if(ExoP.RecItem(15) == 1 && flag)
        if(TransState(position) == 'D' && TransState(position+1) == 'R')
            TransDR= (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                     (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        else
            flag = false;
        end
        position = position+6;
    else
        TransDR = 0;
    end
    
    % Sxxx
    if(ExoP.RecItem(16) == 1 && flag)
        if(TransState(position) == 'S')
            TransMode = (TransState(position+1)-48)*1;
            TransAsym = (TransState(position+2)-48)*1;
            TransTech = (TransState(position+3)-48)*1;
        else
            flag = false;
        end
    else
        TransMode = 0;
        TransAsym = 0;
        TransTech = 0;
    end
    
end

if flag
    ExoP.TransTime = [ExoP.TransTime; toc];
    ExoP.torqueTL = [ExoP.torqueTL;TransTorqueTL];
    ExoP.angleAL = [ExoP.angleAL;TransAngleAL];
    ExoP.angleVL = [ExoP.angleVL;TransAngleVL];
    ExoP.angleACL = [ExoP.angleACL;TransAngleACCL];
    ExoP.torqueTR = [ExoP.torqueTR;TransTorqueTR];
    ExoP.angleAR = [ExoP.angleAR;TransAngleAR];
    ExoP.angleVR = [ExoP.angleVR;TransAngleVR];
    ExoP.angleACR = [ExoP.angleACR;TransAngleACCR];
    ExoP.angleP = [ExoP.angleP;TransAngleP];
    ExoP.angleY = [ExoP.angleY;TransAngleY];
    ExoP.adotPV = [ExoP.adotPV;TransVeloV];
    ExoP.PWM_L = [ExoP.PWM_L;TransCL];
    ExoP.PWM_R = [ExoP.PWM_R;TransCR];
    ExoP.DesiredTorque = [ExoP.DesiredTorque;[TransDL,TransDR]];
    ExoP.MotionMode = [ExoP.MotionMode;[ExoP.NoTrans, TransMode, TransAsym, TransTech]];
    if ExoP.MotionMode(end,2) ~= ExoP.MotionMode(end-1,2)
        ExoP.MotionMode(end,1) = ExoP.StateTrans;
        if ExoP.MotionMode(end,2) == ExoP.Standing && ExoP.MotionMode(end-1,2) == ExoP.Lifting
            ExoP.BendCycle = ExoP.BendCycle + 1;
        end
    end
%     % JUST NEEDED FOR SOME STRATEGY: Other info from calculaion
%     if size(ExoP.adotPV,1) == 1 % Fisrt cycle
%         Trans_tdotTL = 0;
%         Trans_tdotTR = 0;
%         Trans_adotAL = ExoP.adotPV; 
%         Trans_adotAR = ExoP.adotPV;
%     else
%         % Calculation
%         delta_t = ExoP.TransTime(end)-ExoP.TransTime(end-1);
%         Trans_tdotTL = (ExoP.torqueTL(end)-ExoP.torqueTL(end-1))/delta_t;
%         Trans_tdotTR = (ExoP.torqueTR(end)-ExoP.torqueTR(end-1))/delta_t;
%         Trans_adotAL = (ExoP.angleAL(end)-ExoP.angleAL(end-1))/delta_t;
%         Trans_adotAR = (ExoP.angleAR(end)-ExoP.angleAR(end-1))/delta_t;
%     end
%     ExoP.tdotTL = [ExoP.tdotTL;Trans_tdotTL];
%     ExoP.tdotTR = [ExoP.tdotTR;Trans_tdotTR];
%     ExoP.adotAL = [ExoP.adotAL;Trans_adotAL];
%     ExoP.adotAR = [ExoP.adotAR;Trans_adotAR];
    
    % TransState
    ExoP.Temp = [ExoP.Temp; TransStateTemp];
    TempApp.txtInfo.Value = TransState; 
else
    Control_Update = 0;
    Send_Update = 0;
end

%%%%%%%%%%%%%%% Here can add more data decompostition processing %%%%%%%%%%%%%

end