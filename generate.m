clear all; close all; clc;

%% Symbolic variables
% relative joint angles
syms q1 q2 q3 q4 q5 real
syms dq1 dq2 dq3 dq4 dq5 real
syms ddq1 ddq2 ddq3 ddq4 ddq5 real
% absolute pelvis coordinates
syms f1x f1y df1x df1y real

% gravity and tim
syms g t real
% link lengths
syms L_torso L_fem L_tib real
% link masses
syms M_torso M_fem M_tib real
% center of mass offsets
syms Lz_torso Lz_fem Lz_tib real
% link inertias
syms I_torso I_fem I_tib real
% put them into a vector
p = [g L_torso L_fem L_tib M_torso M_fem M_tib I_torso I_fem I_tib Lz_torso Lz_fem Lz_tib]';

%% State arrays
qs = [q1 q2 q3 q4 q5]';
dqs = [dq1 dq2 dq3 dq4 dq5]';
ddqs = [ddq1 ddq2 ddq3 ddq4 ddq5]';

x = [qs; dqs];

qe = [q1 q2 q3 q4 q5 f1x f1y]';
dqe = [dq1 dq2 dq3 dq4 dq5 df1x df1y]';

%% Absolute joint coordinates
q1L = q5;
q31L = q5 + q1;
q32L = q5 + q2;
q41L = q5 + q1 + q3;
q42L = q5 + q2 + q4;

dq1L = jacobian(q1L,qs)*dqs;
dq31L = jacobian(q31L,qs)*dqs;
dq32L = jacobian(q32L,qs)*dqs;
dq41L = jacobian(q41L,qs)*dqs;
dq42L = jacobian(q42L,qs)*dqs;

matlabFunction([q31L q32L q41L q42L q1L dq31L dq32L dq41L dq42L dq1L]','Vars',{qs,dqs},'File','generated/q_to_qL');

%% Stance model
pfoot1 = [0; 0];
pknee1 =          Rot(q41L - pi/2)*[L_tib; 0];
phip   = pknee1 + Rot(q31L - pi/2)*[L_fem; 0];
pknee2 = phip   + Rot(q32L + pi/2)*[L_fem; 0];
pfoot2 = pknee2 + Rot(q42L + pi/2)*[L_tib; 0];

pfem1  = phip   + Rot(q31L + pi/2)*[Lz_fem; 0];
pfem2  = phip   + Rot(q32L + pi/2)*[Lz_fem; 0];
ptib1  = pknee1 + Rot(q41L + pi/2)*[Lz_tib; 0];
ptib2  = pknee2 + Rot(q42L + pi/2)*[Lz_tib; 0];
ptorso = phip   + Rot(q1L + pi/2)*[Lz_torso; 0];

pCoM = (M_fem*(pfem1 + pfem2) + M_tib*(ptib1 + ptib2) + ...
        M_torso*ptorso)/(2*M_fem + 2*M_tib + M_torso);
vCoM = simplify(jacobian(pCoM,qs)*dqs);
% matlabFunction(pCoM,'Vars',{qs,p},'File','generated/pCoM')
% matlabFunction(vCoM,'Vars',{qs,dqs,p},'File','generated/vCoM')

vfoot1 = [0; 0];
vknee1 = jacobian(pknee1,qs)*dqs;
vhip   = jacobian(phip,qs)*dqs;
vknee2 = jacobian(pknee2,qs)*dqs;
vfoot2 = jacobian(pfoot2,qs)*dqs;

vfem1 = jacobian(pfem1,qs)*dqs;
vfem2 = jacobian(pfem2,qs)*dqs;
vtib1 = jacobian(ptib1,qs)*dqs;
vtib2 = jacobian(ptib2,qs)*dqs;
vtorso = jacobian(ptorso,qs)*dqs;

pJoints = [pfoot1 pknee1 phip pknee2 pfoot2];
vJoints = [vfoot1 vknee1 vhip vknee2 vfoot2];
pBodies = [pfem1 ptib1 ptorso ptib2 pfem2];
vBodies = [vfem1 vtib1 vtorso vtib2 vfem2];

Jc = jacobian(vfoot2,dqs);
dJcdt = Jc;
for i = 1:size(Jc,1)
    for ii = 1:size(Jc,2)
        dJcdt(i,ii) = jacobian(Jc(i,ii),qs)*dqs;
    end
end
Jc = simplify(Jc);
dJcdt = simplify(dJcdt);
% matlabFunction(Jc,'Vars',{qs,p},'File','generated/Jc_matrix.m')
% matlabFunction(dJcdt,'Vars',{qs,dqs,p},'File','generated/dJcdt_matrix.m')


JG = jacobian(vCoM,dqs);
Jzeta = jacobian(vfoot2,dqs);
Jq5 = jacobian(dq1L,dqs);
Jtss = [JG; Jzeta; Jq5];
dJtssdt = Jtss;
for i = 1:size(Jtss,1)
    for ii = 1:size(Jtss,2)
        dJtssdt(i,ii) = jacobian(Jtss(i,ii),qs)*dqs;
    end
end
Jtss = simplify(Jtss);
dJtssdt = simplify(dJtssdt);
% matlabFunction(Jtss,'Vars',{qs,p},'File','generated/Jtss_matrix.m')
% matlabFunction(dJtssdt,'Vars',{qs,dqs,p},'File','generated/dJtssdt_matrix.m')


%% Kinetic and potential energy
Ekin = 1/2*M_fem*(vfem1'*vfem1 + vfem2'*vfem2) + ...
       1/2*M_tib*(vtib1'*vtib1 + vtib2'*vtib2) + ...
       1/2*M_torso*(vtorso'*vtorso) + ...
       1/2*I_fem*(dq31L'*dq31L + dq32L'*dq32L) + ...
       1/2*I_tib*(dq41L'*dq41L + dq42L'*dq42L) + ...
       1/2*I_torso*(dq1L'*dq1L);
   
Epot = M_fem*g*(pfem1(2) + pfem2(2)) + ...
       M_tib*g*(ptib1(2) + ptib2(2)) + ...
       M_torso*g*(ptorso(2));
   
%% Generate system matrices
N = length(qs);

G_vect = simplify(jacobian(Epot,qs).');

for i = 1:N
    for j = 1:N
        D_mtx(i,j) = diff(diff(Ekin,dqs(i)),dqs(j));
    end
end
D_mtx = simplify(D_mtx);

for j=1:N
	for k=1:N
		for i=1:N
			Ctemp(i) = (diff(D_mtx(k,j),qs(i))+diff(D_mtx(k,i),qs(j))-diff(D_mtx(i,j),qs(k)))*dqs(i);
        end
        C_mtx(k,j) = 1/2*sum(Ctemp);
	end
end
C_mtx=simplify(C_mtx);

B_mtx = sym([eye(N-1); zeros(1,N-1)]);

%% Some control arrays/matrices
syms q1d zHd z2d k1 k2 k3 k4 real
k = [k1 k2 k3 k4]';
d = [q1d zHd z2d]';
syms inv_D_mtx11 inv_D_mtx12 inv_D_mtx13 inv_D_mtx14 inv_D_mtx15 real
syms inv_D_mtx21 inv_D_mtx22 inv_D_mtx23 inv_D_mtx24 inv_D_mtx25 real
syms inv_D_mtx31 inv_D_mtx32 inv_D_mtx33 inv_D_mtx34 inv_D_mtx35 real
syms inv_D_mtx41 inv_D_mtx42 inv_D_mtx43 inv_D_mtx44 inv_D_mtx45 real
syms inv_D_mtx51 inv_D_mtx52 inv_D_mtx53 inv_D_mtx54 inv_D_mtx55 real
inv_D_mtx_sym = [inv_D_mtx11 inv_D_mtx12 inv_D_mtx13 inv_D_mtx14 inv_D_mtx15;
                 inv_D_mtx21 inv_D_mtx22 inv_D_mtx23 inv_D_mtx24 inv_D_mtx25;
                 inv_D_mtx31 inv_D_mtx32 inv_D_mtx33 inv_D_mtx34 inv_D_mtx35;
                 inv_D_mtx41 inv_D_mtx42 inv_D_mtx43 inv_D_mtx44 inv_D_mtx45;
                 inv_D_mtx51 inv_D_mtx52 inv_D_mtx53 inv_D_mtx54 inv_D_mtx55];

% d1 = phip(1) - pfoot1(1);
% d2 = phip(1) - pfoot2(1);
% 
% H = [k1*(q1L - q1d);
%      k2*(d1 + d2);
%      k3*(phip(2) - zHd);
%      k4*(pfoot2(2) - z2d)];
%  
% dHdt   = jacobian(H,qs)*dqs;
% dHdq   = simplify(jacobian(H,qs));
% dHdtdq = simplify(jacobian(dHdq*dqs,qs));
% LfH =   [dHdq zeros(4,5)]*[dqs;
%                            inv_D_mtx_sym*(-C_mtx*dqs - G_vect)];
% LfLfH = [dHdtdq dHdq]*[dqs;
%                                       inv_D_mtx_sym*(-C_mtx*dqs - G_vect)];
% LgLfH = dHdq*inv_D_mtx_sym*B_mtx;
% 
% LfH = simplify(LfH);
% LfLfH = simplify(LfLfH);
% LgLfH = simplify(LgLfH);

%% Output definitions for nonlinear 
% 5-link has 4 actuators so need to define 4 desired outputs
% y = [Lst
%      betast
%      Lsw
%      q5]
syms Lst_des betast_des Lsw_des betasw_des q5_des real
syms dLst_des dbetast_des dLsw_des dbetasw_des dq5_des real
syms ddLst_des ddbetast_des ddLsw_des ddbetasw_des ddq5_des real
syms alpha11 alpha12 alpha13 alpha14 alpha15 real
syms alpha21 alpha22 alpha23 alpha24 alpha25 real
syms alpha31 alpha32 alpha33 alpha34 alpha35 real
syms alpha41 alpha42 alpha43 alpha44 alpha45 real

alpha = [alpha11 alpha12 alpha13 alpha14 alpha15;
         alpha21 alpha22 alpha23 alpha24 alpha25;
         alpha31 alpha32 alpha33 alpha34 alpha35;
         alpha41 alpha42 alpha43 alpha44 alpha45];
Ydes = sym(zeros(4,1));
% dYdes = sym(zeros(4,1));
for i = 1:4
    for ai = 1:size(alpha,2)
        Ydes(i) = Ydes(i) + alpha(i,ai)*pCoM(1)^(size(alpha,2)-ai);
%         dYdes(i) = dYdes(i) + dalpha(i,ai)*pCoM(1)^(size(alpha,2)-ai);
    end
end
        
% dYdes = jacobian(Ydes,qs)*dqs;
% ddYdes = jacobian(dYdes,qs)*dqs + jacobian(dYdes,dqs)*ddqs;

% Ydes = [Lst_des Lsw_des betasw_des q5_des]';
% dYdes = [dLst_des dLsw_des dbetasw_des dq5_des]';
% ddYdes = [ddLst_des ddLsw_des ddbetasw_des ddq5_des]';

Lst = norm(phip);
betast = atan2(phip(1),phip(2));
Lsw = norm(pfoot2 - phip);
betasw = -atan2(phip(1)-pfoot2(1),phip(2)-pfoot2(2));

dLst = norm(vhip);
dbetast = atan2(vhip(1),vhip(2));
dLsw = norm(vfoot2 - vhip);
dbetasw = -atan2(vhip(1)-vfoot2(1),vhip(2)-vfoot2(2));

Y = [Lst Lsw betasw q5]';
dY = [dLst dLsw dbetasw dq5]';

h = Y - Ydes;

f_vector = simplify([dqs;
     inv_D_mtx_sym*(-C_mtx*dqs - G_vect)]);
g_vector = simplify([sym(zeros(5,4));
     inv_D_mtx_sym*B_mtx]);
 
dhdq = jacobian(h - Ydes,[qs; dqs]);

Lfh = dhdq*f_vector;
LgLfh = jacobian(Lfh,x);    % misses *f and *g, too large sym file
LfLfh = jacobian(Lfh,x);

% matlabFunction(h,'Vars',{qs,alpha,p},'File','generated/h_vector')
% matlabFunction(f_vector,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/f_vector')
% matlabFunction(g_vector,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/g_vector')
% matlabFunction(Lfh,'Vars',{qs,dqs,alpha,inv_D_mtx_sym,p},'File','generated/Lfh_vector')
% matlabFunction(LfLfh,'Vars',{qs,dqs,alpha,inv_D_mtx_sym,p},'File','generated/LfLfh_vector')
% matlabFunction(LgLfh,'Vars',{qs,dqs,alpha,inv_D_mtx_sym,p},'File','generated/LgLfh_matrix')

%% Y = based on zcom
f_vector = simplify([dqs;
     inv_D_mtx_sym*(-C_mtx*dqs - G_vect)]);
g_vector = simplify([sym(zeros(5,4));
     inv_D_mtx_sym*B_mtx]);

Ya = simplify([pCoM(2) pfoot2(1) pfoot2(2) q5]');
dYa = jacobian(Ya,qs)*dqs;
% ddYa = jacobian(dYa,qs)*dqs + jacobian(dYa,dqs)*ddqs;

Lfh = simplify(jacobian(Ya,[qs;dqs])*f_vector);
LfLfh_nof = jacobian(Lfh,[qs;dqs]);
LgLfh_nog = jacobian(Lfh,[qs;dqs]);

% matlabFunction(Ya,'Vars',{qs,dqs,p},'File','generated/SSPSLIP/Ya_SSP_vector')
% matlabFunction(dYa,'Vars',{qs,dqs,p},'File','generated/SSPSLIP/dYa_SSP_vector')
% matlabFunction(ddYa,'Vars',{qs,dqs,ddqs,p},'File','generated/SSPSLIP/ddYa_vector')

% matlabFunction(Lfh,'Vars',{qs,dqs,p},'File','generated/SSPSLIP/Lfh_SSP_vector')
% matlabFunction(LfLfh_nof,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/SSPSLIP/LfLfh_nof_SSP_vector')
% matlabFunction(LgLfh_nog,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/SSPSLIP/LgLfh_nog_SSP_matrix')

% matlabFunction(f,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/f_vector')
% matlabFunction(g,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/g_vector')

%% TSC-QP
Y = [q1 q2 q3 q4]';
dY = [dq1 dq2 dq3 dq4]';

bq = inv_D_mtx_sym*(-C_mtx*dqs - G_vect);
Aq = inv_D_mtx_sym*(B_mtx);

Jy = simplify(jacobian(Y,qs));
dJy = simplify(jacobian(Jy*dqs,qs));

% tau = [tau1; tau2; tau3; tau4];
% 
% ddY = Fx + Gx*tau;
% matlabFunction(Y,'Vars',{qs,p},'File','generated/Y_vector')
% matlabFunction(Jy,'Vars',{qs,p},'File','generated/Jy_matrix')
% matlabFunction(dJy,'Vars',{qs,dqs,p},'File','generated/dJy_matrix')
% matlabFunction(Aq,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/Aq_matrix')
% matlabFunction(bq,'Vars',{qs,dqs,inv_D_mtx_sym,p},'File','generated/bq_vector')

%% Create functions
% matlabFunction(D_mtx,'Vars',{qs,p},'File','generated/D_matrix')
% matlabFunction(C_mtx,'Vars',{qs,dqs,p},'File','generated/C_matrix')
% matlabFunction(G_vect,'Vars',{qs,p},'File','generated/G_vector')
% matlabFunction(B_mtx,'Vars',{},'File','generated/B_matrix')

% matlabFunction(pJoints,'Vars',{qs,p},'File','generated/pJoints');
% matlabFunction(pBodies,'Vars',{qs,p},'File','generated/pBodies');
% matlabFunction(vJoints,'Vars',{qs,dqs,p},'File','generated/vJoints');
% matlabFunction(vBodies,'Vars',{qs,dqs,p},'File','generated/vBodies');

% matlabFunction(H,'Vars',{qs,dqs,k,d,p},'File','generated/H_vector.m')
% matlabFunction(dHdt,'Vars',{qs,dqs,k,d,p},'File','generated/dHdt_vector.m')
% matlabFunction(dHdq,'Vars',{qs,dqs,k,d,p},'File','generated/dHdq_matrix.m')
% matlabFunction(dHdtdq,'Vars',{qs,dqs,k,d,p},'File','generated/dHdtdq_matrix.m')

%% Impact model
pfoot1 = [f1x; f1y];
pknee1 = pfoot1 + Rot(q41L - pi/2)*[L_tib; 0];
phip   = pknee1 + Rot(q31L - pi/2)*[L_fem; 0];
pknee2 = phip   + Rot(q32L + pi/2)*[L_fem; 0];
pfoot2 = pknee2 + Rot(q42L + pi/2)*[L_tib; 0];

pfem1  = phip   + Rot(q31L + pi/2)*[Lz_fem; 0];
pfem2  = phip   + Rot(q32L + pi/2)*[Lz_fem; 0];
ptib1  = pknee1 + Rot(q41L + pi/2)*[Lz_tib; 0];
ptib2  = pknee2 + Rot(q42L + pi/2)*[Lz_tib; 0];
ptorso = phip   + Rot(q1L + pi/2)*[Lz_torso; 0];

pCoM = (M_fem*(pfem1 + pfem2) + M_tib*(ptib1 + ptib2) + ...
        M_torso*ptorso)/(2*M_fem + 2*M_tib + M_torso);
    
vfoot1 = [0; 0];
vknee1 = jacobian(pknee1,qe)*dqe;
vhip   = jacobian(phip,qe)*dqe;
vknee2 = jacobian(pknee2,qe)*dqe;
vfoot2 = jacobian(pfoot2,qe)*dqe;

vfem1 = jacobian(pfem1,qe)*dqe;
vfem2 = jacobian(pfem2,qe)*dqe;
vtib1 = jacobian(ptib1,qe)*dqe;
vtib2 = jacobian(ptib2,qe)*dqe;
vtorso = jacobian(ptorso,qe)*dqe;

pJointse = [pfoot1 pknee1 phip pknee2 pfoot2];
vJointse = [vfoot1 vknee1 vhip vknee2 vfoot2];
pBodiese = [pfem1 ptib1 ptorso ptib2 pfem2];
vBodiese = [vfem1 vtib1 vtorso vtib2 vfem2];

%% Kinetic and potential energy
Ekin = 1/2*M_fem*(vfem1'*vfem1 + vfem2'*vfem2) + ...
       1/2*M_tib*(vtib1'*vtib1 + vtib2'*vtib2) + ...
       1/2*M_torso*(vtorso'*vtorso) + ...
       1/2*I_fem*(dq31L'*dq31L + dq32L'*dq32L) + ...
       1/2*I_tib*(dq41L'*dq41L + dq42L'*dq42L) + ...
       1/2*I_torso*(dq1L'*dq1L);
   
Epot = M_fem*g*(pfem1(2) + pfem2(2)) + ...
       M_tib*g*(ptib1(2) + ptib2(2)) + ...
       M_torso*g*(ptorso(2));
   
%% Generate system matrices
Ge_vect = simplify(jacobian(Epot,qe).');

for i = 1:N
    for j = 1:N
        De_mtx(i,j) = diff(diff(Ekin,dqe(i)),dqe(j));
    end
end
De_mtx = simplify(De_mtx);

N = length(qs);
for j=1:N
	for k=1:N
		for i=1:N
			Ctemp(i) = (diff(De_mtx(k,j),qs(i))+diff(De_mtx(k,i),qs(j))-diff(De_mtx(i,j),qs(k)))*dqs(i);
        end
        Ce_mtx(k,j) = 1/2*sum(Ctemp);
	end
end
Ce_mtx=simplify(Ce_mtx);

Ee_mtx = jacobian(pfoot2,qs);

%% Create functions
matlabFunction(De_mtx,'Vars',{qs,p},'File','generated/De_matrix')
matlabFunction(Ce_mtx,'Vars',{qs,dqs,p},'File','generated/Ce_matrix')
matlabFunction(Ge_vect,'Vars',{qs,p},'File','generated/Ge_vector')
matlabFunction(Ee_mtx,'Vars',{qs,p},'File','generated/Ee_matrix')

matlabFunction(pJointse,'Vars',{qe,p},'File','generated/pJointse');
matlabFunction(pBodiese,'Vars',{qe,p},'File','generated/pBodiese');
matlabFunction(vJointse,'Vars',{qe,dqe,p},'File','generated/vJointse');
matlabFunction(vBodiese,'Vars',{qe,dqe,p},'File','generated/vBodiese');

%% Helper functions
function output = Rot(x)
    output = [cos(x) -sin(x); sin(x) cos(x)];
end