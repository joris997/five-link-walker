function dx = evaluateEoM(t,x,p,alphas,thetamp,H0,c,controller,param)
global failure

q = x(1:5);
dq = x(6:10);

%% Equations of motion (dx = Fx + Gx*u)
D_mtx = D_matrix(q,p);
C_mtx = C_matrix(q,dq,p);
G_vec = G_vector(q,p);
B_mtx = B_matrix();

Fvec = -C_mtx*dq-G_vec;
Fx = D_mtx\(-C_mtx*dq-G_vec);
Gx = D_mtx\(B_mtx);

Kp = param.KP;
Kd = param.KD;
eps = param.eps;

switch controller
    % Input-output linearization controller
    case 'IO'
        u = IO_controller(x,alphas,thetamp,H0,c,B_mtx,eps,Kd,Kp,D_mtx,Fvec);
        
    % RES-CLF controller
    case 'CLF_QP'
        u = CLF_QP_controller(x,alphas,thetamp,H0,c,B_mtx,eps,D_mtx,Fvec);
        
    % TSC controllers
    case 'TSC_QP'
        u = TSC_QP_controller(x,p,alphas,thetamp,H0,c,B_mtx,Kd,Kp,D_mtx,Fvec);

    % No torque applied
    case '0'
        u = zeros(4,1);
end

dx = [dq;
      Fx + Gx*u];
  
if any(isnan(u))
    failure = true;
    error('observed nan');
end
end