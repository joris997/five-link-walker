function [qeplus,dqeplus,impulse] = impactMapping(qeminus,dqeminus,p)

ndof = length(dqeminus);

De = De_matrix(qeminus,p);
Ee = Ee_matrix(qeminus,p);

R = [0 1 0 0 0; 
     1 0 0 0 0; 
     0 0 0 1 0; 
     0 0 1 0 0; 
     0 0 0 0 1];
 
% impact map, see Grizzle's book, section 3.4.2, eq 3.21
P  = [De, -Ee'; 
      Ee, zeros(2,2)]\[De * dqeminus; 
                       zeros(2,1)]; %22x1

qeplus = R*qeminus;
dqeplus = R*P(1:ndof);
impulse = P(ndof+1:end);

end
