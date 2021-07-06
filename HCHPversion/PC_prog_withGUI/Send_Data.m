function Send_Data()
% Since program running to here, the necessary information of latest time
% have stored in P
% PC to MCU Protocol: TLxxxxTRxxxxMxx\r\n (0x0D,0x0A)
global ExoP;
global TempApp;
% Here we mainly want to send mode and desired torque to MCU
McuSerial = ExoP.config{1,1};
MotionMode = ExoP.MotionMode(end,:); 
DesiredTorque = roundn(ExoP.DesiredTorque(end,:),-2);  % Keep it to two decimal places

%% Combining the sending data
% Here the specific form for sending data is designed as: "TLxxxxTRxxxx"
TransState = 'TL';
if DesiredTorque(1) == 0
    TransState = [TransState,'0000'];
elseif DesiredTorque(1) >0 && DesiredTorque(1) < 1
    TransState = [TransState,'00',num2str(DesiredTorque(1)*100)];
elseif DesiredTorque(1) >=1 && DesiredTorque(1) < 10
    TransState = [TransState,'0',num2str(DesiredTorque(1)*100)];
else
    TransState = [TransState,num2str(DesiredTorque(1)*100)];
end
TransState = [TransState,'TR'];   
if DesiredTorque(2) == 0
    TransState = [TransState,'0000'];
elseif DesiredTorque(2) >0 && DesiredTorque(2) < 1
    TransState = [TransState,'00',num2str(DesiredTorque(2)*100)];
elseif DesiredTorque(2) >=1 && DesiredTorque(2) < 10
    TransState = [TransState,'0',num2str(DesiredTorque(2)*100)];
else
    TransState = [TransState,num2str(DesiredTorque(2)*100)];
end

TransState = [TransState,'M',num2str(MotionMode(2)-1),num2str(MotionMode(3))];
TempApp.txtCommand.Value = TransState;
% % FOR TEST ONLY
% TransState = 'TL0000TR0000M10';
%% Send data to serial port
flushoutput(McuSerial);      % flush the output buffer
fprintf(McuSerial,TransState);    % send the data
flushinput(McuSerial);       % flush the input buffer for next receiving cycle
% wait until data are all sent
% while McuSerial.BytesToOutput ~= 0  
% end

end