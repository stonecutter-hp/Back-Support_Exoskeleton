function Control()
global ExoP;
%% Conducting corresponding UID strategy as selected in GUI
% NOTICE FEEDBACK FROM DIFFERENT UID STRATEGY SHOULD KEEP IDENTICAL FOR THE
% EASE OF PROGRAM MAINTENANCE
switch ExoP.UIDStrategy
    case 1  % Basic UID Strategy
        mode = BasicUIDStrategy();         
    case 2  % Developed UID Strategy
        % Can be replaced for future developed UID strategy by 
        % mode = DevelopedUIDStrategy();
        mode = [ExoP.NoTrans, ExoP.Exit, ExoP.None+ExoP.fricEnable*ExoP.fricCompen];      % ONLY FOR INDICATION
    case 0  % Test mode
        mode = [ExoP.NoTrans, ExoP.Exit, ExoP.None+ExoP.fricEnable*ExoP.fricCompen];      % ONLY FOR INDICATION
    otherwise
        mode = [ExoP.NoTrans, ExoP.Exit, ExoP.None+ExoP.fricEnable*ExoP.fricCompen];      % ONLY FOR INDICATION       
end

%% Conducting corresponding RTG strategy as selected in GUI
% NOTICE FEEDBACK FROM DIFFERENT RTG STRATEGY SHOULD KEEP IDENTICAL FOR THE
% EASE OF PROGRAM MAINTENANCE
switch ExoP.RTGStrategy
    case 1  % Impedance Strategy
        DesiredTorque = RTGImpedanceStra(mode);
    case 2  % Compensation Strategy
        DesiredTorque = RTGCompensationStra(mode);
    case 3  % Velocity-based RTG Strategy
        % Can be replaced for future developed UID strategy by 
        % DesiredTorque = RTGVelBasedStra();
        DesiredTorque = [0, 0];  % ONLY FOR INDICATION
    case 0  % Test mode
        DesiredTorque = [0, 0];  % ONLY FOR INDICATION
    otherwise
        DesiredTorque = [0, 0];  % ONLY FOR INDICATION
end

%% Saving necessary information
% mode is 1*3 vector: mode(1) is the motion transition flag, mode(2) is
% motion state, mode(3) refer to left/right asymmetric + friction
% compensation 
ExoP.MotionMode = [ExoP.MotionMode;mode];           
% DesiredTorque is 1*2 vector: (1) for left, (2) for right
ExoP.DesiredTorque = [ExoP.DesiredTorque;DesiredTorque]; 
end