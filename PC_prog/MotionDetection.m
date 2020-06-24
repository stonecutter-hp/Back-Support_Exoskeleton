function [mode,ConInf] = MotionDetection()
% the initial motion detection algorithm can refer to Presentaion_0317 P25
% ---- 2020-0331
global P;
Alpha_Thre = P.Alpha_Thre;
AlphaDot_Thre = P.AlphaDot_Thre;
Beta_Thre = P.Beta_Thre;
%% Calculate the referenced information for both motion detection and desired torque generation
% angle difference
AngleDiff = P.angleAL(end) - P.angleAR(end);   
% average of angle sum
AngleMean = (P.angleAL(end) + P.angleAR(end))/2;
% average of speed sum
AngleDot = (P.angleSL(end) + P.angleSR(end))/2;
% yaw angle
YawAngle = P.angleY(end);
% Here we can calculate standard deviation over last P.Range times  
[RawNum,ColNum] = size(P.angleAL);             % get the number of sensor information  have getten until now
% Get certain range of sensor information
if (ColNum < P.Range)
    TempAngleL = [zeros(1,(P.Range-ColNum)),P.angleL(1:end)];
else
    TempAngleL = P.angleL((ColNum-P.Range+1):end);
end
% ThetaStd = ....

%% Make use of the above sensor information to infer motion mode of human based on designed algorithm
if abs(AngleMean) < Alpha_Thre
    mode(1) = 1;   % Other motion
else
    if abs(AngleDot) < AlphaDot_Thre
        if abs(YawAngle) < Beta_Thre
            mode(1) = 2;    %Symmetric Holding
        else
            mode(1) = 3;    %Asymmetric Holding
        end
    else
        if abs(YawAngle) >= Beta_Thre
            if sign(AngleDot) > 0
                mode(1) = 4;    % Asymmetric Lowering
            else
                mode(1) = 5;    % Asymmetric Lifting
            end
        else
            if sign(AngleDot) > 0
                mode(1) = 6;    % Symmetric Lowering
            else
                mode(1) = 7;    % Symmetric Lifting
            end
        end
            
    end
end

% initial classification of left/right asymmetric
if sign(YawAngle) == 0 
    mode(2) = 0;     % no aysmmetric
elseif sign(YawAngle) > 0;
    mode(2) = 1;     % left
else
    mode(2) = 2;     % right
end

%% Necessary information for torque generation
% refer to Presentaion_0317 P28
ConInf(1) = AngleMean;      % alpha
ConInf(2) = AngleDot;       % alpha_dot
ConInf(3) = YawAngle;       % beta
end