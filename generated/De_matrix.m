function De_mtx = De_matrix(in1,in2)
%DE_MATRIX
%    DE_MTX = DE_MATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 17:07:41

I_fem = in2(9,:);
I_tib = in2(10,:);
I_torso = in2(8,:);
L_fem = in2(3,:);
L_tib = in2(4,:);
Lz_fem = in2(12,:);
Lz_tib = in2(13,:);
Lz_torso = in2(11,:);
M_fem = in2(6,:);
M_tib = in2(7,:);
M_torso = in2(5,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
t2 = cos(q1);
t3 = cos(q3);
t4 = cos(q4);
t5 = q1+q3;
t6 = L_fem.^2;
t7 = L_tib.^2;
t8 = Lz_fem.^2;
t9 = Lz_tib.^2;
t11 = -q1;
t12 = -q2;
t13 = -q4;
t14 = L_fem.*Lz_fem.*M_fem.*2.0;
t15 = L_tib.*Lz_tib.*M_tib.*2.0;
t10 = cos(t5);
t16 = M_tib.*t6;
t17 = M_torso.*t6;
t18 = M_torso.*t7;
t19 = M_fem.*t8;
t20 = M_tib.*t9;
t21 = L_fem.*L_tib.*M_tib.*t3;
t22 = L_fem.*L_tib.*M_torso.*t3;
t23 = L_tib.*Lz_fem.*M_fem.*t3;
t24 = L_fem.*Lz_tib.*M_tib.*t4;
t25 = L_fem.*Lz_torso.*M_torso.*t2;
t26 = -t14;
t27 = -t15;
t28 = M_fem.*t6.*2.0;
t29 = M_fem.*t7.*2.0;
t30 = M_tib.*t7.*2.0;
t31 = q1+t12;
t33 = t5+t12;
t34 = q2+q4+t11;
t35 = L_fem.*L_tib.*M_fem.*t3.*2.0;
t36 = L_fem.*L_tib.*M_fem.*t3.*4.0;
t32 = cos(t31);
t37 = t21.*2.0;
t38 = t22.*2.0;
t39 = t23.*2.0;
t40 = t24.*2.0;
t41 = -t23;
t43 = -t25;
t44 = cos(t33);
t45 = cos(t34);
t46 = L_tib.*Lz_torso.*M_torso.*t10;
t49 = t13+t33;
t55 = I_tib+t20+t24;
t42 = -t39;
t47 = -t46;
t48 = L_fem.*Lz_fem.*M_fem.*t32;
t50 = cos(t49);
t51 = t16.*t32;
t52 = L_fem.*L_tib.*M_tib.*t44;
t53 = L_tib.*Lz_fem.*M_fem.*t44;
t54 = L_fem.*Lz_tib.*M_tib.*t45;
t66 = I_tib+t18+t20+t21+t22+t27+t29+t30+t35+t41;
t56 = -t48;
t57 = -t51;
t58 = -t52;
t59 = -t53;
t60 = -t54;
t61 = L_tib.*Lz_tib.*M_tib.*t50;
t62 = -t61;
t63 = t60+t62;
t65 = t58+t59+t62;
t64 = t55+t63;
t67 = t56+t57+t58+t59+t63;
t69 = t47+t65+t66;
t68 = I_fem+I_tib+t16+t19+t20+t40+t67;
t70 = I_fem+I_tib+t16+t17+t18+t19+t20+t26+t27+t28+t29+t30+t36+t37+t38+t42+t43+t47+t67;
De_mtx = reshape([I_fem+I_tib+t16+t17+t18+t19+t20+t26+t27+t28+t29+t30+t36+t37+t38+t42,t67,t66,t63,t70,t67,I_fem+I_tib+t16+t19+t20+t40,t65,t55,t68,t66,t65,I_tib+t18+t20+t27+t29+t30,t62,t69,t63,t55,t62,I_tib+t20,t64,t70,t68,t69,t64,I_fem.*2.0+I_tib.*2.0+I_torso+t16.*2.0+t17+t18+t19.*2.0+t20.*2.0-t25.*2.0+t26+t27+t28+t29+t30+t36+t37+t38+t40+t42-t46.*2.0-t48.*2.0-t51.*2.0-t52.*2.0-t53.*2.0-t54.*2.0-t61.*2.0+Lz_torso.^2.*M_torso],[5,5]);