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

%% Decompose the data from MCU
% Here the specific form for recieved data is designed as: "AxxxxSxxxxRxxxxPxxxxYxxxx\r\n"
% Angle xxxx + Speed xxxx + IMU xxxx (Roll/Pitch/Yaw) + Torque xxxx + Control command xxxx
if(TransState(1) == 'A')
    TransAngleAL =  (TransState(2)-48)*1+(TransState(3)-48)*0.1+(TransState(4)-48)*0.01+(TransState(5)-48)*0.001;
    P.angleAL = [P.angleAL, TransAngleAL];
end
if(TransState(6) == 'S')
    TransAngleSL =  (TransState(7)-48)*1+(TransState(8)-48)*0.1+(TransState(9)-48)*0.01+(TransState(10)-48)*0.001;
    P.angleSL = [P.angleSL, TransAngleSL];
end
if(TransState(11) == 'R')
    TransAngleR = (TransState(12)-48)*100+(TransState(13)-48)*10+(TransState(14)-48)*1+(TransState(15)-48)*0.1;
    P.angleR = [P.angleR, TransAngleR];
end
if(TransState(16) == 'P')
    TransAngleP =  (TransState(17)-48)*100+(TransState(18)-48)*10+(TransState(19)-48)*1+(TransState(20)-48)*0.1;
    P.angleP = [P.angleP, TransAngleP];
end
if(TransState(21) == 'Y')
    TransAngleY =  (TransState(22)-48)*100+(TransState(23)-48)*10+(TransState(24)-48)*1+(TransState(25)-48)*0.1;
    P.angleY = [P.angleY, TransAngleY];
end

Control_Update = 1;
Send_Update = 1;
%%%%%%%%%%%%%%% Here add more data decompostition processing %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end