function h = h_vector(in1,in2,in3)
%H_VECTOR
%    H = H_VECTOR(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 13:25:29

L_fem = in3(3,:);
L_tib = in3(4,:);
Lz_fem = in3(12,:);
Lz_tib = in3(13,:);
Lz_torso = in3(11,:);
M_fem = in3(6,:);
M_tib = in3(7,:);
M_torso = in3(5,:);
alpha11 = in2(1);
alpha12 = in2(5);
alpha13 = in2(9);
alpha14 = in2(13);
alpha15 = in2(17);
alpha21 = in2(2);
alpha22 = in2(6);
alpha23 = in2(10);
alpha24 = in2(14);
alpha25 = in2(18);
alpha31 = in2(3);
alpha32 = in2(7);
alpha33 = in2(11);
alpha34 = in2(15);
alpha35 = in2(19);
alpha41 = in2(4);
alpha42 = in2(8);
alpha43 = in2(12);
alpha44 = in2(16);
alpha45 = in2(20);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = M_fem.*2.0;
t3 = M_tib.*2.0;
t4 = pi./2.0;
t5 = -t4;
t6 = q5+t4;
t7 = M_torso+t2+t3;
t8 = cos(t6);
t9 = q1+t6;
t10 = q2+t6;
t16 = q1+q5+t5;
t27 = 1.0./t7;
t11 = cos(t9);
t12 = cos(t10);
t13 = q3+t9;
t14 = q4+t10;
t15 = sin(t10);
t17 = cos(t16);
t18 = q3+t16;
t19 = Lz_torso.*t8;
t28 = t27.^2;
t29 = t27.^3;
t20 = cos(t13);
t21 = cos(t14);
t22 = sin(t14);
t23 = L_fem.*t12;
t24 = Lz_fem.*t11;
t25 = Lz_fem.*t12;
t26 = L_fem.*t15;
t30 = t28.^2;
t31 = cos(t18);
t32 = L_fem.*t17;
t33 = Lz_tib.*t20;
t34 = Lz_tib.*t21;
t35 = L_tib.*t22;
t36 = t32.*2.0;
t37 = L_tib.*t31;
t38 = t37.*2.0;
t39 = t19+t32+t37;
t40 = M_torso.*t39;
t41 = t24+t25+t36+t38;
t43 = t23+t32+t33+t34+t38;
t42 = M_fem.*t41;
t44 = M_tib.*t43;
t45 = t40+t42+t44;
t46 = t45.^2;
t47 = t45.^3;
t48 = t46.^2;
h = [-alpha15+sqrt(abs(t32+t37).^2+abs(L_fem.*sin(t16)+L_tib.*sin(t18)).^2)-alpha14.*t27.*t45-alpha13.*t28.*t46-alpha12.*t29.*t47-alpha11.*t30.*t48;-alpha25+sqrt(abs(t26+t35).^2+abs(t23+L_tib.*t21).^2)-alpha24.*t27.*t45-alpha23.*t28.*t46-alpha22.*t29.*t47-alpha21.*t30.*t48;-alpha35-atan2(-t23-L_tib.*t21,-t26-t35)-alpha34.*t27.*t45-alpha33.*t28.*t46-alpha32.*t29.*t47-alpha31.*t30.*t48;-alpha45+q5-alpha44.*t27.*t45-alpha43.*t28.*t46-alpha42.*t29.*t47-alpha41.*t30.*t48];
