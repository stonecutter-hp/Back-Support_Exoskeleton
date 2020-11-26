function [Control_Update,Send_Update] = Receive_McuData()
global P;
%% Read data from MCU
McuPort1 = P.config{1,1};
flushinput(McuPort1);
TransState = fscanf(McuPort1);   % read the input buffer
% ensure the data completeness
while numel(TransState) ~= P.ReceiveDataNum
%    flushinput(McuPort1);
    TransState = fscanf(McuPort1);
end

%% Decompose the data from MCU from characters to numbers
% Here the specific form for recieved data is designed as: 
% TLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxx\r\n
% TL/Rxxxx: (Nm) Torsion spring torque for left/right transmission system 
% LL/Rxxxx: (N) Load cell for cable force of left/right transmission system
% AL/Rxxxxx: (deg) Potentiometer/IMU feedback for angle between left/right
%                  thigh and vertical direction
%            first number indicate sign: 0 for -, 1 for +
% Pxxxxx: (deg) Pitch angle for trunk
%               first number indicate sign: 0 for -, 1 for +
% Yxxxxx: (deg) Yaw angle for trunk
%               first number indicate sign: 0 for -, 1 for +
% Vxxxxx: (deg/s) Pitch angular velocity for trunk
%                 first number indicate sign: 0 for -, 1 for +
position = 1;
% TLxxxx
if(P.RecItem(1) == 1)
    if(TransState(position) == 'T' && TransState(position+1) == 'L')
        TransTorqueTL = (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                        (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        P.torqueTL = [P.torqueTL, TransTorqueTL];
    end
    position = position+6;
end
% LLxxxx
if(P.RecItem(2) == 1)
    if(TransState(position) == 'L' && TransState(position+1) == 'L')
        TransForceLL = (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                       (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        P.forceLL = [P.forceLL, TransForceLL];
    end
    position = position+6;
end
% ALxxxxx
if(P.RecItem(3) == 1)
    if(TransState(position) == 'A' && TransState(position+1) == 'L')
        TransAngleAL = (TransState(position+3)-48)*10+(TransState(position+4)-48)*1+...
                       (TransState(position+5)-48)*0.1+(TransState(position+6)-48)*0.01;
        if(TransState(position+2) == '0')
            TransAngleAL = -1*TransAngleAL;
        end
        P.angleAL = [P.angleAL, TransAngleAL];
    end
    position = position+7;
end
% TRxxxx
if(P.RecItem(4) == 1)
    if(TransState(position) == 'T' && TransState(position+1) == 'R')
        TransTorqueTR = (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                        (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        P.torqueTR = [P.torqueTR, TransTorqueTR];
    end
    position = position+6;
end
% LRxxxx
if(P.RecItem(5) == 1)
    if(TransState(position) == 'L' && TransState(position+1) == 'R')
        TransForceLR = (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                       (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        P.forceLR = [P.forceLR, TransForceLR];
    end
    position = position+6;
end
% ARxxxxx
if(P.RecItem(6) == 1)
    if(TransState(position) == 'A' && TransState(position+1) == 'R')
        TransAngleAR = (TransState(position+3)-48)*10+(TransState(position+4)-48)*1+...
                       (TransState(position+5)-48)*0.1+(TransState(position+6)-48)*0.01;
        if(TransState(position+2) == '0')
            TransAngleAR = -1*TransAngleAR;
        end
        P.angleAR = [P.angleAR, TransAngleAR];
    end
    position = position+7;
end
% Pxxxxx
if(P.RecItem(7) == 1)
    if(TransState(position) == 'P')
        TransAngleP = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                      (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
        if(TransState(position+1) == '0')
            TransAngleP = -1*TransAngleP;
        end    
        P.angleP = [P.angleP, TransAngleP];
    end
    position = position+6;
end
% Yxxxxx
if(P.RecItem(8) == 1)
    if(TransState(position) == 'Y')
        TransAngleY = (TransState(position+2)-48)*10+(TransState(position+3)-48)*1+...
                      (TransState(position+4)-48)*0.1+(TransState(position+5)-48)*0.01;
        if(TransState(position+1) == '0')
            TransAngleY = -1*TransAngleY;
        end
        P.angleY = [P.angleY, TransAngleY];
    end
    position = position+6;
end
%Vxxxxx
if(P.RecItem(9) == 1)
    if(TransState(position) == 'V')
        TransVeloV = (TransState(position+2)-48)*100+(TransState(position+3)-48)*10+...
                     (TransState(position+4)-48)*1+(TransState(position+5)-48)*0.1;
        if(TransState(position+1) == '0')
            TransVeloV = -1*TransVeloV;
        end
        P.adotPV = [P.adotPV, TransVeloV];
    end
end
%%%%%%%%%%%%%%% Here can add more data decompostition processing %%%%%%%%%%%%%

% After receiving, allow next control and send cycle
Control_Update = 1;
Send_Update = 1;

end