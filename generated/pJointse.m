function pJointse = pJointse(in1,in2)
%PJOINTSE
%    PJOINTSE = PJOINTSE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 17:07:46

L_fem = in2(3,:);
L_tib = in2(4,:);
f1x = in1(6,:);
f1y = in1(7,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q2+q5+t2;
t5 = cos(t4);
t6 = q4+t4;
t7 = sin(t4);
t8 = q1+q5+t3;
t9 = cos(t8);
t10 = q3+t8;
t11 = sin(t8);
t12 = L_fem.*t5;
t13 = L_fem.*t7;
t14 = cos(t10);
t15 = sin(t10);
t16 = L_fem.*t9;
t17 = L_fem.*t11;
t18 = L_tib.*t14;
t19 = L_tib.*t15;
pJointse = reshape([f1x,f1y,f1x+t18,f1y+t19,f1x+t16+t18,f1y+t17+t19,f1x+t12+t16+t18,f1y+t13+t17+t19,f1x+t12+t16+t18+L_tib.*cos(t6),f1y+t13+t17+t19+L_tib.*sin(t6)],[2,5]);
