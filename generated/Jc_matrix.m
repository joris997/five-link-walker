function Jc = Jc_matrix(in1,in2)
%JC_MATRIX
%    JC = JC_MATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 13:24:52

L_fem = in2(3,:);
L_tib = in2(4,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = q1+q5;
t3 = q2+q5;
t4 = cos(t2);
t5 = cos(t3);
t6 = q3+t2;
t7 = q4+t3;
t8 = sin(t2);
t9 = sin(t3);
t10 = cos(t6);
t11 = cos(t7);
t12 = sin(t6);
t13 = sin(t7);
t14 = L_fem.*t4;
t15 = L_fem.*t5;
t16 = L_fem.*t8;
t17 = L_fem.*t9;
t18 = L_tib.*t10;
t19 = L_tib.*t11;
t20 = L_tib.*t12;
t21 = L_tib.*t13;
t22 = -t15;
t23 = -t17;
t24 = -t19;
t25 = -t21;
Jc = reshape([t14+t18,t16+t20,t22+t24,t23+t25,t18,t20,t24,t25,t14+t18+t22+t24,t16+t20+t23+t25],[2,5]);
