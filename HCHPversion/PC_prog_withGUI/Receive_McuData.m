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
% MxTLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxx\r\n
% Mx: Marking flag to show if the MCU data is real-time with successful 
%     receiving last command from PC
% TL/Rxxxx: (Nm) Torsion spring torque for left/right transmission system 
% LL/Rxxxx: (N) Load cell for cable force of left/right transmission system
% AL/Rxxxxx: (deg) Potentiometer feedback for hip angle
%            first number indicate sign: 0 for -, 1 for +
% Pxxxxx: (deg) Pitch angle for trunk
%               first number indicate sign: 0 for -, 1 for +
% Yxxxxx: (deg) Yaw angle for trunk
%               first number indicate sign: 0 for -, 1 for +
% Vxxxxx: (deg/s) Pitch angular velocity for trunk
%                 first number indicate sign: 0 for -, 1 for +
position = 3;
flag = true;
% Also some security operation can be added if over P.MaxDelay loop no 
% command is recieved from PC like: stop(P.config{2,1});

if(TransState(position-2) == 'M' && flag)
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