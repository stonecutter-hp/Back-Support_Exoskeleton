function P = SubjectInit(SubjectNum)
% This function is to initialize the biomechanical parameter for different
% subjects
if SubjectNum == 1
    P = HongPengData();
elseif SubjectNum == 2
    P = HugoData();
else
    P = HongPengData();
end

end
