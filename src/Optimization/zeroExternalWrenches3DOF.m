function ZEW = zeroExternalWrenches3DOF(N)
%ZEROEXTERNALWRENCHES3DOF returns zero external wrenches variable of size N
    ZEW.FX = zeros(3, N);
    ZEW.FY = zeros(3, N);
    ZEW.FZ = zeros(3, N);
    ZEW.CX = zeros(3, N);
    ZEW.CY = zeros(3, N);
    ZEW.CZ = zeros(3, N);
end