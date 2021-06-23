function P = testSet_Parameters()
% This function is to set parameters for the testing program

%% Timer initialization parameter
P.MainFreq = 1;
P.MaxRunTime = 1;

%% Timer loop running indicator
% Corresponds to the sensor info from low-level control and human motion
% mode from UID strategy
P.runIndicator = 0;
P.data = [];
%% Configuration information
% P.config{1,1} for serial port configuration
% P.config{2,1} for timer configuration
P.config = cell(2,1);
% Program stop Flag: 0-normal auto stop; 1-normal manually stop
%                    2-cannot open serial port; 3-handshake error; 
%                    4-serial communication error
P.stopFlag = 0;
%% Controller mode selection
% Negative sign indicator enable flag, corresponds to the friction
% compensation enable flag 
P.negEable = 0;  % 0-disable, 1-enable

% Algorithm selection flag, corresponds to the UID/RTG strategy selection
% flag 
P.alogMode = 1; % 1-alg 1; 2-alg 2; 3-alg 3; 4-alg 4


end