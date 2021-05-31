% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : C:\Users\139783\Desktop\6DoF_model\6DOF.dyn




%      Geometric parameters   

% j     ant   mu    sigma gamma b     alpha d     theta r
% 1     0     0     2     0     0     0     0     0     0
% 2     1     1     0     0     0     0     0     q1    0
% 3     2     1     0     0     0     0     L1    q2    0
% 4     3     1     0     0     0     0     L2    q3    0
% 5     4     1     0     0     0     0     L3    q4    0
% 6     5     1     0     0     0     0     L4    q5    0
% 7     6     1     0     0     0     0     L5    q6    0



%              Inertial parameters

% j     XX    XY    XZ    YY    YZ    ZZ    MX    MY    MZ    M     Ia

% 1     0     0     0     0     0     0     0     0     0     0     0

% 2     XX1   XY1   XZ1   YY1   YZ1   ZZ1   MX1   MY1   MZ1   M1    0

% 3     XX2   XY2   XZ2   YY2   YZ2   ZZ2   MX2   MY2   MZ2   M2    0

% 4     XX3   XY3   XZ3   YY3   YZ3   ZZ3   MX3   MY3   MZ3   M3    0

% 5     XX4   XY4   XZ4   YY4   YZ4   ZZ4   MX4   MY4   MZ4   M4    0

% 6     XX5   XY5   XZ5   YY5   YZ5   ZZ5   MX5   MY5   MZ5   M5    0

% 7     XX6   XY6   XZ6   YY6   YZ6   ZZ6   MX6   MY6   MZ6   M6    0



%  External forces,friction parameters, joint velocities and accelerations

% j      FX     FY     FZ     CX     CY     CZ     FS     FV     QP     QDP

% 1      FX1    FY1    FZ1    CX1    CY1    CZ1    0      0      0      0

% 2      0      0      0      0      0      0      0      0      dq1    ddq1

% 3      0      0      0      0      0      0      0      0      dq2    ddq2

% 4      0      0      0      0      0      0      0      0      dq3    ddq3

% 5      0      0      0      0      0      0      0      0      dq4    ddq4

% 6      0      0      0      0      0      0      0      0      dq5    ddq5

% 7      FX7    FY7    FZ7    CX7    CY7    CZ7    0      0      dq6    ddq6

% Base velocity, base accelerations, and gravity

% j     W0    WP0   V0    VP0   G

% 1     0     0     0     0     0

% 2     0     0     0     0     G2

% 3     0     0     0     0     0

%  Dynamic model: Newton Euler method
% Equations:

% Declaration of the function
function [GAMMA, EN0] = Dyn_6DOF(q,dq,ddq,modelParam,extWrenches)
    
    % Declare gravitational force
    G2 = 9.81;

    % Extract Parameters
    for ii = 1 : 6
        % Extract joint angles, velocities and accelerations
        eval(['q' num2str(ii) '=q(' num2str(ii) ',:);']);
        eval(['dq' num2str(ii) '=dq(' num2str(ii) ',:);']);
        eval(['ddq' num2str(ii) '=ddq(' num2str(ii) ',:);']);
        
        % Multiply the segment COM positions by segment mass
        modelParam.(['COM' num2str(ii)])=modelParam.(['COM' num2str(ii)])*modelParam.(['M' num2str(ii)]);
        % Segment mass
        eval(['M' num2str(ii) '=modelParam.M' num2str(ii) ';']);
        % Segment COM X
        eval(['MX' num2str(ii) '=modelParam.COM' num2str(ii) '(1);']);
        % Segment COM Y
        eval(['MY' num2str(ii) '=modelParam.COM' num2str(ii) '(2);']);
        % Segment COM Z to 0
        eval(['MZ' num2str(ii) '=0;']);
        % Segment Lenghts
        eval(['L' num2str(ii) '=modelParam.L' num2str(ii) ';']);
        % Segment Moments of Inertia
        eval(['ZZ' num2str(ii) '=modelParam.ZZ' num2str(ii) ';']);
        % Set Mixed Moments of Inertia to 0
        eval(['XZ' num2str(ii) '=0;']);
        eval(['YZ' num2str(ii) '=0;']);        
    end

FX7=extWrenches.EndEffectorWrenches(1, :);
FY7=extWrenches.EndEffectorWrenches(2, :);
FZ7=extWrenches.EndEffectorWrenches(3, :);
CX7=extWrenches.EndEffectorWrenches(4, :);
CY7=extWrenches.EndEffectorWrenches(5, :);
CZ7=extWrenches.EndEffectorWrenches(6, :);

FX1=extWrenches.ModelBaseWrenches(1, :);
FY1=extWrenches.ModelBaseWrenches(2, :);
FZ1=extWrenches.ModelBaseWrenches(3, :);
CX1=extWrenches.ModelBaseWrenches(4, :);
CY1=extWrenches.ModelBaseWrenches(5, :);
CZ1=extWrenches.ModelBaseWrenches(6, :);

% Function description:
	S2=sin(q1);
	C2=cos(q1);
	S3=sin(q2);
	C3=cos(q2);
	S4=sin(q3);
	C4=cos(q3);
	S5=sin(q4);
	C5=cos(q4);
	S6=sin(q5);
	C6=cos(q5);
	S7=sin(q6);
	C7=cos(q6);
	DV332=-dq1.^2;
	VP12=-(G2.*S2);
	VP22=-(C2.*G2);
	F12=DV332.*MX1 - ddq1.*MY1 + M1.*VP12;
	F22=ddq1.*MX1 + DV332.*MY1 + M1.*VP22;
	No12=ddq1.*XZ1 + DV332.*YZ1;
	No22=-(DV332.*XZ1) + ddq1.*YZ1;
	No32=ddq1.*ZZ1;
	W33=dq1 + dq2;
	WP33=ddq1 + ddq2;
	DV333=-W33.^2;
	VSP13=DV332.*L1 + VP12;
	VSP23=ddq1.*L1 + VP22;
	VP13=C3.*VSP13 + S3.*VSP23;
	VP23=-(S3.*VSP13) + C3.*VSP23;
	F13=DV333.*MX2 + M2.*VP13 - MY2.*WP33;
	F23=DV333.*MY2 + M2.*VP23 + MX2.*WP33;
	No13=WP33.*XZ2 + DV333.*YZ2;
	No23=-(DV333.*XZ2) + WP33.*YZ2;
	No33=WP33.*ZZ2;
	W34=dq3 + W33;
	WP34=ddq3 + WP33;
	DV334=-W34.^2;
	VSP14=DV333.*L2 + VP13;
	VSP24=VP23 + L2.*WP33;
	VP14=C4.*VSP14 + S4.*VSP24;
	VP24=-(S4.*VSP14) + C4.*VSP24;
	F14=DV334.*MX3 + M3.*VP14 - MY3.*WP34;
	F24=DV334.*MY3 + M3.*VP24 + MX3.*WP34;
	No14=WP34.*XZ3 + DV334.*YZ3;
	No24=-(DV334.*XZ3) + WP34.*YZ3;
	No34=WP34.*ZZ3;
	W35=dq4 + W34;
	WP35=ddq4 + WP34;
	DV335=-W35.^2;
	VSP15=DV334.*L3 + VP14;
	VSP25=VP24 + L3.*WP34;
	VP15=C5.*VSP15 + S5.*VSP25;
	VP25=-(S5.*VSP15) + C5.*VSP25;
	F15=DV335.*MX4 + M4.*VP15 - MY4.*WP35;
	F25=DV335.*MY4 + M4.*VP25 + MX4.*WP35;
	No15=WP35.*XZ4 + DV335.*YZ4;
	No25=-(DV335.*XZ4) + WP35.*YZ4;
	No35=WP35.*ZZ4;
	W36=dq5 + W35;
	WP36=ddq5 + WP35;
	DV336=-W36.^2;
	VSP16=DV335.*L4 + VP15;
	VSP26=VP25 + L4.*WP35;
	VP16=C6.*VSP16 + S6.*VSP26;
	VP26=-(S6.*VSP16) + C6.*VSP26;
	F16=DV336.*MX5 + M5.*VP16 - MY5.*WP36;
	F26=DV336.*MY5 + M5.*VP26 + MX5.*WP36;
	No16=WP36.*XZ5 + DV336.*YZ5;
	No26=-(DV336.*XZ5) + WP36.*YZ5;
	No36=WP36.*ZZ5;
	W37=dq6 + W36;
	WP37=ddq6 + WP36;
	DV337=-W37.^2;
	VSP17=DV336.*L5 + VP16;
	VSP27=VP26 + L5.*WP36;
	VP17=C7.*VSP17 + S7.*VSP27;
	VP27=-(S7.*VSP17) + C7.*VSP27;
	F17=DV337.*MX6 + M6.*VP17 - MY6.*WP37;
	F27=DV337.*MY6 + M6.*VP27 + MX6.*WP37;
	No17=WP37.*XZ6 + DV337.*YZ6;
	No27=-(DV337.*XZ6) + WP37.*YZ6;
	No37=WP37.*ZZ6;
	E17=F17 + FX7;
	E27=F27 + FY7;
	N17=CX7 + No17 - MZ6.*VP27;
	N27=CY7 + No27 + MZ6.*VP17;
	N37=CZ7 + No37 - MY6.*VP17 + MX6.*VP27;
	FDI17=C7.*E17 - E27.*S7;
	FDI27=C7.*E27 + E17.*S7;
	E16=F16 + FDI17;
	E26=F26 + FDI27;
	N16=C7.*N17 + No16 - N27.*S7 - MZ5.*VP26;
	N26=-(FZ7.*L5) + C7.*N27 + No26 + N17.*S7 + MZ5.*VP16;
	N36=FDI27.*L5 + N37 + No36 - MY5.*VP16 + MX5.*VP26;
	FDI16=C6.*E16 - E26.*S6;
	FDI26=C6.*E26 + E16.*S6;
	E15=F15 + FDI16;
	E25=F25 + FDI26;
	N15=C6.*N16 + No15 - N26.*S6 - MZ4.*VP25;
	N25=-(FZ7.*L4) + C6.*N26 + No25 + N16.*S6 + MZ4.*VP15;
	N35=FDI26.*L4 + N36 + No35 - MY4.*VP15 + MX4.*VP25;
	FDI15=C5.*E15 - E25.*S5;
	FDI25=C5.*E25 + E15.*S5;
	E14=F14 + FDI15;
	E24=F24 + FDI25;
	N14=C5.*N15 + No14 - N25.*S5 - MZ3.*VP24;
	N24=-(FZ7.*L3) + C5.*N25 + No24 + N15.*S5 + MZ3.*VP14;
	N34=FDI25.*L3 + N35 + No34 - MY3.*VP14 + MX3.*VP24;
	FDI14=C4.*E14 - E24.*S4;
	FDI24=C4.*E24 + E14.*S4;
	E13=F13 + FDI14;
	E23=F23 + FDI24;
	N13=C4.*N14 + No13 - N24.*S4 - MZ2.*VP23;
	N23=-(FZ7.*L2) + C4.*N24 + No23 + N14.*S4 + MZ2.*VP13;
	N33=FDI24.*L2 + N34 + No33 - MY2.*VP13 + MX2.*VP23;
	FDI13=C3.*E13 - E23.*S3;
	FDI23=C3.*E23 + E13.*S3;
	E12=F12 + FDI13;
	E22=F22 + FDI23;
	N12=C3.*N13 + No12 - N23.*S3 - MZ1.*VP22;
	N22=-(FZ7.*L1) + C3.*N23 + No22 + N13.*S3 + MZ1.*VP12;
	N32=FDI23.*L1 + N33 + No32 - MY1.*VP12 + MX1.*VP22;
	FDI12=C2.*E12 - E22.*S2;
	FDI22=C2.*E22 + E12.*S2;
	E11=FDI12 + FX1;
	E21=FDI22 + FY1;
	E31=FZ1 + FZ7;
	N11=CX1 + C2.*N12 - N22.*S2;
	N21=CY1 + C2.*N22 + N12.*S2;
	N31=CZ1 + N32;
    
% 	GAM2=N32;
% 	GAM3=N33;
% 	GAM4=N34;
% 	GAM5=N35;
% 	GAM6=N36;
% 	GAM7=N37;
% 	E10=E11;
% 	E20=E21;
% 	E30=E31;
% 	N10=N11;
% 	N20=N21;
% 	N30=N31;
    
    GAMMA = [N32; N33; N34; N35; N36; N37];
    EN0   = [E11; E21; E31; N11; N21; N31];


% *=*
% Number of operations : 160 '+' or '-', 186 '*' or '/'
