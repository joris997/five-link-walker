function G_vect = G_vector(in1,in2)
%G_VECTOR
%    G_VECT = G_VECTOR(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 14:28:37

L_fem = in2(3,:);
L_tib = in2(4,:);
Lz_fem = in2(12,:);
Lz_tib = in2(13,:);
Lz_torso = in2(11,:);
M_fem = in2(6,:);
M_tib = in2(7,:);
M_torso = in2(5,:);
g = in2(1,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = q1+q5;
t3 = q2+q5;
t4 = q3+t2;
t5 = q4+t3;
t6 = sin(t2);
t7 = sin(t3);
t8 = sin(t4);
t9 = sin(t5);
t10 = Lz_tib.*M_tib.*g.*t9;
t11 = -t10;
G_vect = [L_fem.*M_fem.*g.*t6.*2.0+L_fem.*M_tib.*g.*t6+L_tib.*M_fem.*g.*t8.*2.0+L_fem.*M_torso.*g.*t6+L_tib.*M_tib.*g.*t8.*2.0+L_tib.*M_torso.*g.*t8-Lz_fem.*M_fem.*g.*t6-Lz_tib.*M_tib.*g.*t8;t11-L_fem.*M_tib.*g.*t7-Lz_fem.*M_fem.*g.*t7;g.*t8.*(L_tib.*M_fem.*2.0+L_tib.*M_tib.*2.0+L_tib.*M_torso-Lz_tib.*M_tib);t11;g.*(L_fem.*M_fem.*t6.*2.0+L_fem.*M_tib.*t6-L_fem.*M_tib.*t7+L_tib.*M_fem.*t8.*2.0+L_fem.*M_torso.*t6+L_tib.*M_tib.*t8.*2.0+L_tib.*M_torso.*t8-Lz_fem.*M_fem.*t6-Lz_fem.*M_fem.*t7-Lz_tib.*M_tib.*t8-Lz_tib.*M_tib.*t9-Lz_torso.*M_torso.*sin(q5))];
