close all; clear all; clc;
addpath('generated')
addpath('generated/CBF')
addpath('controllers')

%% Load modelling parameters
p = loadParams;

%% Load optimization results (Min)
load('opt_results.mat');
alphas = params{1, 1}.aposition;
alphas = reshape(alphas,[4,6]);
alphas = -[alphas(1,:);alphas(3,:);alphas(2,:);alphas(4,:)];
thetamp = params{1, 1}.pposition;
clear controller u

qs = states{1,1}.x;
qs = -[qs(4,1);qs(6,1);qs(5,1);qs(7,1);qs(3,1)];
dqs = states{1,1}.dx;
dqs = -[dqs(4,1);dqs(6,1);dqs(5,1);dqs(7,1);dqs(3,1)];

%% Load initial condition
y0 = [-2.5195
      -2.8919
      -0.4728
      -0.4933
      -0.1879
       1.2534
      -0.9456
      -5.6175
      -1.5642
       0.4000];
   
% If you make the joint velocities zero, the walker certainly won't walk
% y0(6:10) = zeros(5,1);
y0(6:10) = 0.5*y0(6:10);

xy = pJointse([y0(1:5);[0;0]],p);
xyB = pBodiese([y0(1:5);[0;0]],p);

figure(99)
clf
hold on;
plot([xy(1,1);xy(1,2)],[xy(2,1);xy(2,2)],'c','LineWidth',3)
plot([xy(1,2);xy(1,3)],[xy(2,2);xy(2,3)],'b','LineWidth',3)

plot([xy(1,3);xyB(1,3)],[xy(2,3);xyB(2,3)],'k','LineWidth',3)

plot([xy(1,3);xy(1,4)],[xy(2,3);xy(2,4)],'g','LineWidth',3)
plot([xy(1,4);xy(1,5)],[xy(2,4);xy(2,5)],'r','LineWidth',3)
yline(0)
xlim([-2 2])
ylim([-1.5 1.5])
drawnow
   
%% Control parameters
H0 = [eye(4) zeros(4,1)];
c = [-1 0 -1/2 0 -1];

eps = .03; 

m = 4;
F = [zeros(m) eye(m); zeros(m) zeros(m)];
G = [zeros(m); eye(m)];
Ieps = diag([1/eps*ones(m,1);ones(m,1)]);
Q = eye(2*m);
P = icare(F,G,Q,[],[],[],[]);
Peps = Ieps*P*Ieps;
K = 1/2*G'*P; %equivalent
KP = K(1:m,1:m);
KD = K(1:m,m+1:end);

contr_params = struct;
contr_params.KP = KP;
contr_params.KD = KD;
contr_params.eps = eps;
contr_params.dist = 0;

%% Make some steps
time = [0]; states = [y0']; stFoot = [0.0 0.0];

opt = odeset('Events',@switchingSurface,...
             'MaxStep',1e-2,...
             'RelTol',1e-5,...
             'AbsTol',1e-5);

% controller = 'IO';
controller = 'CLF_QP';
% controller = 'TSC_QP';

global step downstep failure
failure = false;
downstep = 0.00;
for step = 1:4
    jointPositions = pJoints(y0(1:5),p);
    p_swingxy = jointPositions(:,end);

    [t,qs,~,ye,~] = ode45(@(t,y)evaluateEoM(t,y,p,alphas,thetamp,H0,c,controller,contr_params),...
                   [0.15 5],y0,opt);
    t(end)
    
    % Get coordinates with pelvis coordinate. 
    % qs = joint coordinates
    % qe = [qs; pelvis horizontal and vertical]
    [qe, dqe, imp] = impactMapping(qs(end,1:5)',qs(end,6:10)',p);
    
    y0 = [qe; dqe];
    
    stFoot = [stFoot; 
              stFoot(end,1) - p_swingxy(1)*ones(length(t),1) stFoot(end,2) - p_swingxy(2)*ones(length(t),1)];

    time = [time; time(end) + t];
    states = [states; qs];
    
    if failure
        break;
    end
end

%% plotting
figure
for i = 1:5
    subplot(2,5,i)
    plot(time,states(:,i),'LineWidth',2)
    ylabel('angle [rad]')
    xlabel('time [s]')
    grid on
    
    subplot(2,5,i+5)
    plot(time,states(:,i+5),'LineWidth',2)
    ylabel('dangle [rad/s]')
    xlabel('time [s]')
    grid on
end

idx = find(stFoot(:,2)<-0.001,1);
if isempty(idx)
    idx = length(t);
end

titles = {'st_{hip}','sw_{hip}','st_{knee}','sw_{knee}','pelvis'};
f = figure;
for i = 1:5
    subplot(1,5,i); hold on; grid on;
    plot(states(1:idx,i),states(1:idx,i+5),'b','LineWidth',1)
    plot(states(idx:end,i),states(idx:end,i+5),'b','LineWidth',1)
    ylabel('dangle [rad/s]')
    xlabel('angle [rad]')
    title(titles{i})
end
f.Position = [100 100 900 300];

%% Angular momentum
% order of bodies from pBodies:
%       [tib1 fem1 pelv fem2 tib2]
masses   = [p(7) p(6) p(5) p(6) p(7)];
inertias = [p(10) p(9) p(8) p(9) p(10)];

AngMom = zeros(length(time),3);
for i = 1:length(time)
    qL = q_to_qL(states(i,1:5)',states(i,6:10)');
    phi = -[qL(6) qL(8) qL(7) qL(9) qL(10)];
    
    xyJ  = [pJoints(states(i,1:5)',p); zeros(1,5)];
    xyB  = [pBodies(states(i,1:5)',p); zeros(1,5)];
    vxyJ = [vJoints(states(i,1:5)',states(i,6:10)',p); zeros(1,5)];
    vxyB = [vBodies(states(i,1:5)',states(i,6:10)',p); zeros(1,5)];
    
    H = 0;
    for b = 1:size(xyB,2)
        r = xyB(:,b);
        
        H = H + cross(r,masses(b)*vxyB(:,b)) + inertias(b)*phi(b);
    end
    AngMom(i,:) = H';
end

figure; hold on; grid on;
plot(time,AngMom(:,3))

%% Animation
figure(100)
timei = 0.010;
for i = 1:length(time)
    if time(i) > timei
        xy = pJointse([states(i,1:5)';stFoot(i,:)'],p);
        xyB = pBodiese([states(i,1:5)';stFoot(i,:)'],p);

        figure(100)
        clf
        hold on;
        plot([xy(1,1);xy(1,2)],[xy(2,1);xy(2,2)],'c','LineWidth',3)
        plot([xy(1,2);xy(1,3)],[xy(2,2);xy(2,3)],'b','LineWidth',3)
        
        plot([xy(1,3);xyB(1,3)],[xy(2,3);xyB(2,3)],'k','LineWidth',3)
        
        plot([xy(1,3);xy(1,4)],[xy(2,3);xy(2,4)],'g','LineWidth',3)
        plot([xy(1,4);xy(1,5)],[xy(2,4);xy(2,5)],'r','LineWidth',3)
        yline(0)
        xlim([-2 2])
        ylim([-1.5 1.5])
        drawnow
        
        timei = timei + 0.01;
    end
end

