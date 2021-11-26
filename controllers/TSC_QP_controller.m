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
    variable u(4,1)
%     v = LgLfh*u + Lf2h;
    
    Y    = Y_vect - hd;
    dY  = Jy_mtx*dq - dhdt;
    ddY = Jy_mtx*(Aq_mtx*u + bq_vec) + dJy_mtx*dq  - ddhdt;
    ddYstar = -Kp*Y - Kd*dY;
    % dYdes = ddY2des = -Kp*Y2 - Kd*dY2
    % where
    %   Y2 = Ya2 - Yd2 = h
    %   dY2 = LfY2 - dYd2 = Lfh
    %   Kp = 0.50*eye(4)
    %   Kd = 0.86*eye(4)
%     dYdes = -Kp*Y - Kd*dY;
    
    % Control signal defined; v = LgLfh*u + Lf2h
%     v = LgLfh*u + Lf2h;
    
    minimize( (ddY - ddYstar)'*(ddY - ddYstar) )
%     minimize( (Abar*v + Bbar - dYdes)'*(Abar*v + Bbar - dYdes) )
%     -200*ones(4,1) <= u <= 200*ones(4,1);
cvx_end
u
end

% ddYdes = -Kp*h + -Kd*(Lf2h - ddhdt + LgLfh*u);

% D_mtx = D_matrix(q,p);
% C_mtx = C_matrix(q,dq,p);
% G_vec = G_vector(q,p);
% B_mtx = B_matrix();

% Abar = D_mtx\(B_mtx);
% Bbar = D_mtx\(-C_mtx*dq-G_vec);