function ZEW = zeroExternalWrenches6DOF(N)
%ZEROEXTERNALWRENCHES6DOF returns zero external wrenches variable of size N
    ZEW.EndEffectorWrenches = zeros(6, N);
    ZEW.ModelBaseWrenches = zeros(6, N);
end