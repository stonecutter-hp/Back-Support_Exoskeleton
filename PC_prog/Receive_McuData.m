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
% ALxxxxLLxxxxTLxxxxxARxxxxLRxxxxTRxxxxxPxxxxxYxxxxxVxxxxx\r\n
% AL/Rxxxx: (deg) Support beam angle for left/right transmission system 
% LL/Rxxxx: (N) Load cell for cable force of left/right transmission system
% TL/Rxxxxx: (deg) Potentiometer/IMU feedback for angle between left/right thigh and trunk
%            first number indicate sign: 0 for -, 1 for +
% Pxxxxx: (deg) Pitch angle for trunk
% Yxxxxx: (deg) Yaw angle for trunk
%         first number indicate sign: 0 for -, 1 for +
% Vxxxxx: (deg/s) Pitch angular velocity for trunk
%         first number indicate sign: 0 for -, 1 for +
if(TransState(1) == 'A' && TransState(2) == 'L')
    TransAngleAL = (TransState(3)-48)*10+(TransState(4)-48)*1+(TransState(5)-48)*0.1+(TransState(6)-48)*0.01;
    P.angleAL = [P.angleAL, TransAngleAL];
end
if(TransState(7) == 'L' && TransState(8) == 'L')
    TransForceLL = (TransState(9)-48)*10+(TransState(10)-48)*1+(TransState(11)-48)*0.1+(TransState(12)-48)*0.01;
    P.forceLL = [P.forceLL, TransForceLL];
end
if(TransState(13) == 'T' && TransState(14) == 'L')
    TransAngleTL = (TransState(16)-48)*10+(TransState(17)-48)*1+(TransState(18)-48)*0.1+(TransState(19)-48)*0.01;
    if(TransState(15) == '0')
        TransAngleTL = -1*TransAngleTL;
    end
    P.angleTL = [P.angleTL, TransAngleTL];
end
if(TransState(20) == 'A' && TransState(21) == 'R')
    TransAngleAR = (TransState(22)-48)*10+(TransState(23)-48)*1+(TransState(24)-48)*0.1+(TransState(25)-48)*0.01;
    P.angleAR = [P.angleAR, TransAngleAR];
end
if(TransState(26) == 'L' && TransState(27) == 'R')
    TransForceLR = (TransState(28)-48)*10+(TransState(29)-48)*1+(TransState(30)-48)*0.1+(TransState(31)-48)*0.01;
    P.forceLR = [P.forceLR, TransForceLR];
end
if(TransState(32) == 'T' && TransState(33) == 'R')
    TransAngleTR = (TransState(35)-48)*10+(TransState(36)-48)*1+(TransState(37)-48)*0.1+(TransState(38)-48)*0.01;
    if(TransState(34) == '0')
        TransAngleTR = -1*TransAngleTR;
    end
    P.angleTR = [P.angleTR, TransAngleTR];
end
if(TransState(39) == 'P')
    TransAngleP = (TransState(40)-48)*100+(TransState(41)-48)*10+(TransState(42)-48)*1+(TransState(43)-48)*0.1+(TransState(44)-48)*0.01;
    P.angleP = [P.angleP, TransAngleP];
end
if(TransState(45) == 'Y')
    TransAngleY = (TransState(47)-48)*10+(TransState(48)-48)*1+(TransState(49)-48)*0.1+(TransState(50)-48)*0.01;
    if(TransState(46) == '0')
        TransAngleY = -1*TransAngleY;
    end
    P.angleY = [P.angleY, TransAngleY];
end
if(TransState(51) == 'V')
    TransVeloV = (TransState(53)-48)*100+(TransState(54)-48)*10+(TransState(55)-48)*1+(TransState(56)-48)*0.1;
    if(TransState(52) == '0')
        TransVeloV = -1*TransVeloV;
    end
    P.adotPV = [P.adotPV, TransVeloV];
end
%%%%%%%%%%%%%%% Here can add more data decompostition processing %%%%%%%%%%%%%

% After receiving, allow next control and send cycle
Control_Update = 1;
Send_Update = 1;

end