function [u] = CLF_QP_controller(x,alphas,thetamp,H0,c,B,eps,D_mtx,Fvec)
%% setup & CLF
n = 5;
q = x(1:n);
dq = x(n+1:end);

theta = c*q;

% alphas(3,:) = 0.8*alphas(3,:);
[hd,dhdt,ddhdt] = calc_DesiredOutput(theta,alphas,thetamp);

h = H0*q-hd; % h(q) = h_a(q) - h_d(q)

Lfh = (H0-dhdt*c)*dq;
LgLfh = (H0-dhdt*c)*inv(D_mtx)*B; %size: 4-by-4
Lf2h = -ddhdt*(c*dq)^2+(H0-dhdt*c)*inv(D_mtx)*Fvec; %size: 4-by-1

%% RES-CLF
m = length(h);
eta = [h;Lfh];
F = [zeros(m) eye(m); zeros(m) zeros(m)];
G = [zeros(m); eye(m)];
Ieps = diag([1/eps*ones(m,1);ones(m,1)]);
Q = eye(2*m);
P = icare(F,G,Q,[],[],[],[]);
Peps = Ieps*P*Ieps;
V = eta'*Peps*eta;
LFV = eta'*(F'*Peps+Peps*F)*eta;
LGV = 2*eta'*Peps*G;

%% CLF optimization
cvx_begin quiet
    variable u(4,1)  %v = LgLfh*u+Lf2h;
    variables delta 
    minimize((LgLfh*u+Lf2h)'*(LgLfh*u+Lf2h) + 0.1*delta'*delta )

    (LFV + LGV*(LgLfh*u+Lf2h))<=-1/eps*min(eig(Q))/max(eig(P))*V + delta;
cvx_end
u
end