function [mode,ConInf] = MotionDetection()
% the initial motion detection algorithm can refer to Presentaion_0317 P25
% ---- 2020-0331
% mode(1) - motion type: 1-other motion;       2-Symmetric Holding; 
%                        3-Asymmetric Holding; 4-Asymmetric Lowering;
%                        5-Asymmetric Lifting; 6-Symmetric Lowering;
%                        7-Symmetric Lifting;  0-Stop state
% mode(2) - asymmetric direction: 1-left; 2-right; 0-none.
% ConInf - Alpha, AlphaDot, Beta feedback for reference torque generation
global P;
% User motion detection threshold
Alpha_Thre = P.Alpha_Thre;           % deg
AlphaDot_Thre = P.AlphaDot_Thre;     % deg/s
Beta_Thre = P.Beta_Thre;             % deg
%% Calculate the referenced information for both motion detection and desired torque generation
% Flexion bending angle of trunk
% P.AlphaMean = movmean(P.angleP,5);    % First moving mean is acquired for raw data
P.AlphaMean = P.angleP;
Alpha = P.AlphaMean(end);             % Obtain the alpha for this time's detection
% Flexion bending angular velocity of trunk
cycles = size(P.adotPV,1);
if cycles == 1
    P.AlphaDot = P.adotPV;            % Fisrt cycle's alpha dot
elseif cycles > 1
    % Calculate the alpha dot again in PC
    TransAlphaDot = (P.AlphaMean(end)-P.AlphaMean(end-1))/(P.TransTime(end)-P.TransTime(end-1));
    P.AlphaDot = [P.AlphaDot; TransAlphaDot];
end
% Obtain the alpha dot for this time's detection based on MCU feedback and
% PC calculation results
AlphaDot = (P.AlphaDot(end) + P.adotPV(end))/2; 
% yaw angle
P.BetaMean = movmean(P.angleY,5);     % First moving mean is acquired for raw data
Beta = P.BetaMean(end);               % Obtain the beta for this time's detection


%% Make use of the above sensor information to infer motion mode of human based on designed algorithm
if abs(Alpha) < Alpha_Thre
    mode(1,1) = 1;                % Other motion
else
    if abs(AlphaDot) < AlphaDot_Thre
        if abs(Beta) < Beta_Thre
            mode(1,1) = 2;        % Symmetric Holding
        else
            mode(1,1) = 3;        % Asymmetric Holding
        end
    else
        if abs(Beta) >= Beta_Thre
            if sign(AlphaDot) > 0
                mode(1,1) = 4;    % Asymmetric Lowering
            else
                mode(1,1) = 5;    % Asymmetric Lifting
            end
        else
            if sign(AlphaDot) > 0
                mode(1,1) = 6;    % Symmetric Lowering
            else
                mode(1,1) = 7;    % Symmetric Lifting
            end
        end
            
    end
end

% initial classification of left/right asymmetric
if sign(Beta) == 0 
    mode(1,2) = 0;     % no aysmmetric
elseif sign(Beta) > 0
    mode(1,2) = 1;     % left
else
    mode(1,2) = 2;     % right
end

%% Necessary information for torque generation
% refer to Presentation_20200828
ConInf(1,1) = Alpha;      % alpha
ConInf(2,1) = AlphaDot;   % alpha_dot
ConInf(3,1) = Beta;       % beta
end