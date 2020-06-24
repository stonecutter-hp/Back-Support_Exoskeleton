function Send_Data()
% Since program running to here, the necessary information of last time
% have stroed in P
global P;
% Here we mainly want to send mode and desired torque to MCU
McuPort = P.config{1,1};
% MotionMode = P.MotionMode(end,:); 
DesiredTorque = roundn(P.DesiredTorque(end,:),-2);  % Keep it to two decimal places

%% Combining the sending data
% Here the specific form for sending data is designed as: "TLxxxxTRxxxx"
TransState = 'TL';
if DesiredTorque(1) < 1
    TransState = [TransState,'00',num2str(DesiredTorque(1)*100)];
elseif DesiredTorque(1) >=1 && DesiredTorque(1) < 10
    TransState = [TransState,'0',num2str(DesiredTorque(1)*100)];
else
    TransState = [TransState,num2str(DesiredTorque(1)*100)];
end
TransState = [TransState,'TR'];   
if DesiredTorque(2) < 1
    TransState = [TransState,'00',num2str(DesiredTorque(2)*100)];
elseif DesiredTorque(2) >=1 && DesiredTorque(2) < 10
    TransState = [TransState,'0',num2str(DesiredTorque(2)*100)];
else
    TransState = [TransState,num2str(DesiredTorque(2)*100)];
end
TransState = [TransState,'\r\n'];

%% Send data to serial port
flushoutput(McuPort);      % flush the output buffer
fprintf(McuPort,TransState);    % send the data
% wait until data are all sent
while McuPort.BytesToOutput ~= 0  
end

end