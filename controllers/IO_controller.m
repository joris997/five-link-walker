function [u,v] = IO_controller(x,alphas,thetamp,H0,c,B,eps,KD,KP,Dq,Fvec)

n = 5;
q = x(1:n);
dq = x(n+1:end);

theta = c*q;

[hd,dhdt,ddhdt] = calc_DesiredOutput(theta,alphas,thetamp);
h = H0*q-hd; %h(q) = h_a(q)-h_d(q)

Lfh = (H0-dhdt*c)*dq;
LgLfh = (H0-dhdt*c)*inv(Dq)*B; %size: 4-by-4
Lf2h = -ddhdt*(c*dq)^2+(H0-dhdt*c)*inv(Dq)*Fvec; %size: 4-by-1

v = -(1/eps)*KD*Lfh - (1/(eps^2))*KP*h;
u = inv(LgLfh)*(v-Lf2h);
end