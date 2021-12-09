function P = RTGInit(RTGMode)
% Initialization the specific parameters of RTG strategy according to the
% selection from GUI

if RTGMode == 1
    P = RTGImpedanceStra_Init();
elseif RTGMode == 2
    P = RTGCompensationStra_Init();
elseif RTGMode == 3
    P = RTGVelBasedStra_Init();
elseif RTGMode == 4
    P = RTGImpedanceStra_InitPassive();
else
    P = [];
end

end