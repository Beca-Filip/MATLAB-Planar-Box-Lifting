% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : F:\Dropbox\Research\IOC\PHC_pavle_savic_3DOF\Symoro\3DOF.dyn




%      Geometric parameters   

% j     ant   mu    sigma gamma b     alpha d     theta r
% 1     0     1     0     0     0     0     0     q1    0
% 2     1     1     0     0     0     0     L1    q2    0
% 3     2     1     0     0     0     0     L2    q3    0
% 4     3     1     0     0     0     0     L3    0     0



%              Inertial parameters

% j     XX    XY    XZ    YY    YZ    ZZ    MX    MY    MZ    M     Ia

% 1     0     0     0     0     0     ZZ1   MX1   MY1   0     M1    0

% 2     0     0     0     0     0     ZZ2   MX2   MY2   0     M2    0

% 3     0     0     0     0     0     ZZ3   MX3   MY3   0     M3    0

% 4     0     0     0     0     0     0     0     0     0     0     0



%  External forces,friction parameters, joint velocities and accelerations

% j      FX     FY     FZ     CX     CY     CZ     FS     FV     QP     QDP

% 1      FX1    FY1    0      0      0      CZ1    0      FV1    dq1    ddq1

% 2      FX2    FY2    0      0      0      CZ2    0      FV2    dq2    ddq2

% 3      FX3    FY3    0      0      0      CZ3    0      FV3    dq3    ddq3

% 4      0      0      0      0      0      0      0      0      0      0

% Base velocity, base accelerations, and gravity

% j     W0    WP0   V0    VP0   G

% 1     0     0     0     0     0

% 2     0     0     0     0     G2

% 3     0     0     0     0     0

%  Dynamic model: Newton Euler method
% Equations:

% Declaration of the function
function [GAMMA EN]=Dyn_3DOF(q,dq,ddq,base0,ext_wrenches,param)

% Declaration of %global input variables
q1=q(1,:);
q2=q(2,:);
q3=q(3,:);
dq1=dq(1,:);
dq2=dq(2,:);
dq3=dq(3,:);
ddq1=ddq(1,:);
ddq2=ddq(2,:);
ddq3=ddq(3,:);
G2=base0.G2;MX1=param.MX1;MY1=param.MY1;
 MX2=param.MX2;MY2=param.MY2;
 MX3=param.MX3;MY3=param.MY3;
 L1=param.L1;
L2=param.L2;
M1=param.M1;
M2=param.M2;
M3=param.M3;
 
ZZ1=param.ZZ1;
 
ZZ2=param.ZZ2;
 
ZZ3=param.ZZ3;
 
FV1=param.FV1;
FV2=param.FV2;
FV3=param.FV3;
FX1=ext_wrenches.FX(1,:);FY1=ext_wrenches.FY(1,:);FZ1=ext_wrenches.FZ(1,:);
FX2=ext_wrenches.FX(2,:);FY2=ext_wrenches.FY(2,:);FZ2=ext_wrenches.FZ(2,:);
FX3=ext_wrenches.FX(3,:);FY3=ext_wrenches.FY(3,:);FZ3=ext_wrenches.FZ(3,:);
CX1=ext_wrenches.CX(1,:);CY1=ext_wrenches.CY(1,:);CZ1=ext_wrenches.CZ(1,:);
CX2=ext_wrenches.CX(2,:);CY2=ext_wrenches.CY(2,:);CZ2=ext_wrenches.CZ(2,:);
CX3=ext_wrenches.CX(3,:);CY3=ext_wrenches.CY(3,:);CZ3=ext_wrenches.CZ(3,:);

%global q1 q2 q3 dq1 G2 MX1 ddq1 MY1 M1 ZZ1
%global dq2 ddq2 L1 MX2 M2 MY2 ZZ2 dq3 ddq3 L2
%global MX3 M3 MY3 ZZ3 FX3 FY3 CZ3 FX2 FY2 CZ2
%global FX1 FY1 CZ1 FV1 FV2 FV3

% Declaration of %global output variables
%global GAM1 GAM2 GAM3 GAM4 E10 E20 E30 N10 N20 N30

% Function description:

	S1=sin(q1);
	C1=cos(q1);
	S2=sin(q2);
	C2=cos(q2);
	S3=sin(q3);
	C3=cos(q3);
	DV331=-dq1.^2;
	VP11=-(G2.*S1);
	VP21=-(C1.*G2);
	F11=DV331.*MX1 - ddq1.*MY1 + M1.*VP11;
	F21=ddq1.*MX1 + DV331.*MY1 + M1.*VP21;
	No31=ddq1.*ZZ1;
	W32=dq1 + dq2;
	WP32=ddq1 + ddq2;
	DV332=-W32.^2;
	VSP12=DV331.*L1 + VP11;
	VSP22=ddq1.*L1 + VP21;
	VP12=C2.*VSP12 + S2.*VSP22;
	VP22=-(S2.*VSP12) + C2.*VSP22;
	F12=DV332.*MX2 + M2.*VP12 - MY2.*WP32;
	F22=DV332.*MY2 + M2.*VP22 + MX2.*WP32;
	No32=WP32.*ZZ2;
	W33=dq3 + W32;
	WP33=ddq3 + WP32;
	DV333=-W33.^2;
	VSP13=DV332.*L2 + VP12;
	VSP23=VP22 + L2.*WP32;
	VP13=C3.*VSP13 + S3.*VSP23;
	VP23=-(S3.*VSP13) + C3.*VSP23;
	F13=DV333.*MX3 + M3.*VP13 - MY3.*WP33;
	F23=DV333.*MY3 + M3.*VP23 + MX3.*WP33;
	No33=WP33.*ZZ3;
	E13=F13 + FX3;
	E23=F23 + FY3;
	N33=CZ3 + No33 - MY3.*VP13 + MX3.*VP23;
	FDI13=C3.*E13 - E23.*S3;
	FDI23=C3.*E23 + E13.*S3;
	E12=F12 + FDI13 + FX2;
	E22=F22 + FDI23 + FY2;
	N32=CZ2 + FDI23.*L2 + N33 + No32 - MY2.*VP12 + MX2.*VP22;
	FDI12=C2.*E12 - E22.*S2;
	FDI22=C2.*E22 + E12.*S2;
	E11=F11 + FDI12 + FX1;
	E21=F21 + FDI22 + FY1;
	N31=CZ1 + FDI22.*L1 + N32 + No31 - MY1.*VP11 + MX1.*VP21;
	FDI11=C1.*E11 - E21.*S1;
	FDI21=C1.*E21 + E11.*S1;
	GAM1=dq1.*FV1 + N31;
	GAM2=dq2.*FV2 + N32;
	GAM3=dq3.*FV3 + N33;
	GAM4=0;
	E10=FDI11;
	E20=FDI21;
	E30=0;
	N10=0;
	N20=0;
	N30=N31;


% *=*
% Number of operations : 56 '+' or '-', 61 '*' or '/'
GAMMA(1,:)=GAM1;
GAMMA(2,:)=GAM2;
GAMMA(3,:)=GAM3;
%disp(["l1:", num2str(length(E10)), "l2:", num2str(length(E20)), "l3:", num2str(length(E30)), ...
  %    "l4:", num2str(length(N10)), "l5:", num2str(length(N20)), "l6:", num2str(length(N30))]);
EN(1,:)=E10;EN(2,:)=E20;EN(3,:)=E30;EN(4,:)=N10;EN(5,:)=N20;EN(6,:)=N30;
