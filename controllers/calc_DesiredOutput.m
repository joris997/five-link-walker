function [hd,dhddt,ddhddt] = calc_DesiredOutput(theta,alphas,thetamp)
%theta is either scalar or 1-by-m vector

%phasing variable
tau = (theta-thetamp(2))/(thetamp(1)-thetamp(2));
if tau<0
    tau = 0;
elseif tau>1
    tau = 1;
end

%desired output
m = length(tau);
hd = zeros(4,m);
dhddt = zeros(4,m);
ddhddt = zeros(4,m);
for i = 1:4         % 4 outputs defined
    for j = 1:m     % 1
        hd(i,j) = bezier(0,tau(j),alphas(i,:));
        dhddt(i,j) = bezier(1,tau(j),alphas(i,:))/(thetamp(1)-thetamp(2));
        ddhddt(i,j) = bezier(2,tau(j),alphas(i,:))/(thetamp(1)-thetamp(2))^2;
    end
end

end