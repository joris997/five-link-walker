function g = g_vector(in1,in2,in3,in4)
%G_VECTOR
%    G = G_VECTOR(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 14:32:50

inv_D_mtx11 = in3(1);
inv_D_mtx12 = in3(6);
inv_D_mtx13 = in3(11);
inv_D_mtx14 = in3(16);
inv_D_mtx21 = in3(2);
inv_D_mtx22 = in3(7);
inv_D_mtx23 = in3(12);
inv_D_mtx24 = in3(17);
inv_D_mtx31 = in3(3);
inv_D_mtx32 = in3(8);
inv_D_mtx33 = in3(13);
inv_D_mtx34 = in3(18);
inv_D_mtx41 = in3(4);
inv_D_mtx42 = in3(9);
inv_D_mtx43 = in3(14);
inv_D_mtx44 = in3(19);
inv_D_mtx51 = in3(5);
inv_D_mtx52 = in3(10);
inv_D_mtx53 = in3(15);
inv_D_mtx54 = in3(20);
g = reshape([0.0,0.0,0.0,0.0,0.0,inv_D_mtx11,inv_D_mtx21,inv_D_mtx31,inv_D_mtx41,inv_D_mtx51,0.0,0.0,0.0,0.0,0.0,inv_D_mtx12,inv_D_mtx22,inv_D_mtx32,inv_D_mtx42,inv_D_mtx52,0.0,0.0,0.0,0.0,0.0,inv_D_mtx13,inv_D_mtx23,inv_D_mtx33,inv_D_mtx43,inv_D_mtx53,0.0,0.0,0.0,0.0,0.0,inv_D_mtx14,inv_D_mtx24,inv_D_mtx34,inv_D_mtx44,inv_D_mtx54],[10,4]);
