function pBodiese = pBodiese(in1,in2)
%PBODIESE
%    PBODIESE = PBODIESE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 17:07:46

L_fem = in2(3,:);
L_tib = in2(4,:);
Lz_fem = in2(12,:);
Lz_tib = in2(13,:);
Lz_torso = in2(11,:);
f1x = in1(6,:);
f1y = in1(7,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q5+t2;
t5 = q1+t4;
t6 = q2+t4;
t11 = q1+q5+t3;
t7 = cos(t6);
t8 = q3+t5;
t9 = q4+t6;
t10 = sin(t6);
t12 = cos(t11);
t13 = q3+t11;
t14 = sin(t11);
t15 = cos(t13);
t16 = sin(t13);
t17 = L_fem.*t12;
t18 = L_fem.*t14;
t19 = L_tib.*t15;
t20 = L_tib.*t16;
pBodiese = reshape([f1x+t17+t19+Lz_fem.*cos(t5),f1y+t18+t20+Lz_fem.*sin(t5),f1x+t19+Lz_tib.*cos(t8),f1y+t20+Lz_tib.*sin(t8),f1x+t17+t19+Lz_torso.*cos(t4),f1y+t18+t20+Lz_torso.*sin(t4),f1x+t17+t19+L_fem.*t7+Lz_tib.*cos(t9),f1y+t18+t20+L_fem.*t10+Lz_tib.*sin(t9),f1x+t17+t19+Lz_fem.*t7,f1y+t18+t20+Lz_fem.*t10],[2,5]);