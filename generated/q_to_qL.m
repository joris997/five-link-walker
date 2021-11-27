function out1 = q_to_qL(in1,in2)
%Q_TO_QL
%    OUT1 = Q_TO_QL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 17:06:15

dq1 = in2(1,:);
dq2 = in2(2,:);
dq3 = in2(3,:);
dq4 = in2(4,:);
dq5 = in2(5,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = dq1+dq5;
t3 = dq2+dq5;
t4 = q1+q5;
t5 = q2+q5;
out1 = [t4;t5;q3+t4;q4+t5;q5;t2;t3;dq3+t2;dq4+t3;dq5];
