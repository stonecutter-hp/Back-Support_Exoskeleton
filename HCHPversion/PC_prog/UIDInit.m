function P = UIDInit(UIDMode)
% Initialization the specific parameters of UID strategy according to the
% selection from GUI

if UIDMode == 1
    P = BasicUIDStrategy_Init();
elseif UIDMode == 2
    % P = DevelopedUIDStrategy_Init();
    P = [];
elseif UIDMode == 0
    P = BasicUIDStrategy_Init();
else
    P = [];
end

end