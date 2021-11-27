function [u] = TSC_QP_controller(x,p,alphas,thetamp,H0,c,B,Kd,Kp,D_mtx,Fvec)
%% setup & CLF
n = 5;
q = x(1:n); dq = x(n+1:end);

invD_mtx = inv(D_mtx);

%% Outputs
theta = c*q;
% outputs are relative degree 2
[hd,dhdt,ddhdt] = calc_DesiredOutput(theta,alphas,thetamp);

h = H0*q-hd; % h = Y - Ydes

Lfh = (H0 - dhdt*c)*dq;
LgLfh = (H0 - dhdt*c)*inv(D_mtx)*B;
Lf2h = -ddhdt*(c*dq)^2 + (H0 - dhdt*c)*inv(D_mtx)*Fvec;

%% Affine relation of output derivative and torques
% dYbar = Abar*u + Bbar
% where
%   dYbar = ddY2
% as 
%   ddY2 = Lf2h - ddYd2 + LgLfY2*u
Y_vect = Y_vector(q,p);
Jy_mtx = Jy_matrix(q,p);
dJy_mtx = dJy_matrix(q,dq,p);
Aq_mtx = Aq_matrix(q,dq,invD_mtx,p);
bq_vec = bq_vector(q,dq,invD_mtx,p);

Abar = LgLfh;
Bbar = Lf2h - ddhdt;

%% TSC optimization
cvx_begin quiet
%     variable u(4,1)
%     v = LgLfh*u + Lf2h;
    
%     Y    = Y_vect - hd;
%     dY  = Jy_mtx*dq - dhdt;
%     ddY = Jy_mtx*(Aq_mtx*u + bq_vec) + dJy_mtx*dq  - ddhdt;
%     ddYstar = -Kp*Y - Kd*dY;
% 
%     minimize( (ddY - ddYstar)'*(ddY - ddYstar) )


%     variable v(4,1)
%     dYdes = H0*dq - dhdt;
%     minimize( (Abar*v + Bbar - dYdes)'*(Abar*v + Bbar - dYdes) )


    variable u(4,1)
    eta = [h; Lfh];
    by = ddhdt - dJy_mtx*dq - Kp*h - Kd*Lfh;
    
    H = Jy_mtx'*eye(4)*Jy_mtx;
    g = -Jy_mtx'*eye(4)*by;
    
    minimize( 0.5*u'*H*u + g'*u )
cvx_end
% u = inv(LgLfh)*(v-Lf2h)
u
end

% ddYdes = -Kp*h + -Kd*(Lf2h - ddhdt + LgLfh*u);

% D_mtx = D_matrix(q,p);
% C_mtx = C_matrix(q,dq,p);
% G_vec = G_vector(q,p);
% B_mtx = B_matrix();

% Abar = D_mtx\(B_mtx);
% Bbar = D_mtx\(-C_mtx*dq-G_vec);