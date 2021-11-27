function bq = bq_vector(in1,in2,in3,in4)
%BQ_VECTOR
%    BQ = BQ_VECTOR(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2021 14:28:33

L_fem = in4(3,:);
L_tib = in4(4,:);
Lz_fem = in4(12,:);
Lz_tib = in4(13,:);
Lz_torso = in4(11,:);
M_fem = in4(6,:);
M_tib = in4(7,:);
M_torso = in4(5,:);
dq1 = in2(1,:);
dq2 = in2(2,:);
dq3 = in2(3,:);
dq4 = in2(4,:);
dq5 = in2(5,:);
g = in4(1,:);
inv_D_mtx11 = in3(1);
inv_D_mtx12 = in3(6);
inv_D_mtx13 = in3(11);
inv_D_mtx14 = in3(16);
inv_D_mtx15 = in3(21);
inv_D_mtx21 = in3(2);
inv_D_mtx22 = in3(7);
inv_D_mtx23 = in3(12);
inv_D_mtx24 = in3(17);
inv_D_mtx25 = in3(22);
inv_D_mtx31 = in3(3);
inv_D_mtx32 = in3(8);
inv_D_mtx33 = in3(13);
inv_D_mtx34 = in3(18);
inv_D_mtx35 = in3(23);
inv_D_mtx41 = in3(4);
inv_D_mtx42 = in3(9);
inv_D_mtx43 = in3(14);
inv_D_mtx44 = in3(19);
inv_D_mtx45 = in3(24);
inv_D_mtx51 = in3(5);
inv_D_mtx52 = in3(10);
inv_D_mtx53 = in3(15);
inv_D_mtx54 = in3(20);
inv_D_mtx55 = in3(25);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = sin(q1);
t3 = sin(q3);
t4 = sin(q4);
t5 = sin(q5);
t6 = dq1+dq5;
t7 = dq2+dq5;
t8 = q1+q3;
t9 = q1+q5;
t10 = q2+q5;
t11 = L_fem.^2;
t12 = L_fem.*M_tib;
t13 = L_fem.*M_torso;
t14 = L_tib.*M_torso;
t15 = Lz_fem.*M_fem;
t16 = Lz_tib.*M_tib;
t25 = -q1;
t26 = -q2;
t27 = -q4;
t28 = L_fem.*M_fem.*2.0;
t29 = L_tib.*M_fem.*2.0;
t30 = L_tib.*M_tib.*2.0;
t17 = dq3+t6;
t18 = dq4+t7;
t19 = L_fem.*t4;
t20 = q5+t8;
t21 = q4+t10;
t22 = sin(t8);
t23 = sin(t9);
t24 = sin(t10);
t33 = t3.*t12;
t34 = t3.*t13;
t35 = t3.*t15;
t36 = Lz_torso.*M_torso.*t5;
t39 = -t15;
t40 = -t16;
t41 = q1+t26;
t42 = t3.*t28;
t43 = t8+t26;
t44 = q2+q4+t25;
t46 = L_fem.*L_tib.*M_fem.*t3.*4.0;
t50 = Lz_torso.*t2.*t13.*2.0;
t60 = Lz_tib.*dq4.*t4.*t12;
t61 = Lz_torso.*dq1.*t2.*t13;
t62 = Lz_torso.*dq5.*t2.*t13;
t69 = L_fem.*M_fem.*t3.*-2.0;
t123 = Lz_tib.*dq2.*t4.*t7.*t12;
t31 = sin(t20);
t32 = sin(t21);
t37 = dq2.*t19;
t38 = dq5.*t19;
t45 = sin(t41);
t47 = L_tib.*t33.*2.0;
t48 = L_tib.*t34.*2.0;
t49 = L_tib.*t35.*2.0;
t51 = L_tib.*dq1.*t33;
t52 = L_tib.*dq3.*t33;
t53 = L_tib.*dq5.*t33;
t54 = L_tib.*dq1.*t34;
t55 = L_tib.*dq3.*t34;
t56 = L_tib.*dq5.*t34;
t57 = L_tib.*dq1.*t35;
t58 = L_tib.*dq3.*t35;
t59 = L_tib.*dq5.*t35;
t63 = t12.*t23;
t64 = t12.*t24;
t65 = t13.*t23;
t66 = t15.*t23;
t67 = t15.*t24;
t68 = Lz_torso.*M_torso.*t22;
t70 = -t33;
t71 = -t34;
t72 = -t36;
t81 = -t46;
t84 = sin(t43);
t85 = sin(t44);
t86 = L_tib.*dq1.*t42;
t87 = L_tib.*dq3.*t42;
t88 = L_tib.*dq5.*t42;
t89 = t23.*t28;
t90 = dq2.*t60;
t94 = Lz_torso.*dq1.*t14.*t22;
t95 = Lz_torso.*dq3.*t14.*t22;
t96 = Lz_torso.*dq5.*t14.*t22;
t100 = L_tib.*dq1.*t69;
t101 = L_tib.*dq3.*t69;
t102 = L_tib.*dq5.*t69;
t110 = -t60;
t112 = t23.*t39;
t113 = t24.*t39;
t114 = Lz_torso.*t14.*t22.*2.0;
t115 = L_fem.*M_fem.*g.*t23.*-2.0;
t124 = t18.*t60;
t133 = t27+t43;
t175 = t12+t13+t28+t39;
t176 = t14+t29+t30+t40;
t73 = g.*t63;
t74 = g.*t64;
t75 = g.*t65;
t76 = g.*t66;
t77 = g.*t67;
t78 = t14.*t31;
t79 = t16.*t31;
t80 = t16.*t32;
t82 = -t47;
t83 = -t48;
t91 = g.*t89;
t92 = t29.*t31;
t93 = t30.*t31;
t103 = -t51;
t104 = -t52;
t105 = -t53;
t106 = -t54;
t107 = -t55;
t108 = -t56;
t109 = -t58;
t111 = -t64;
t118 = t31.*t40;
t119 = t32.*t40;
t120 = L_fem.*t85;
t125 = t12.*t84;
t126 = t15.*t84;
t127 = L_tib.*M_fem.*g.*t31.*-2.0;
t128 = L_tib.*M_tib.*g.*t31.*-2.0;
t135 = sin(t133);
t136 = L_fem.*t15.*t45.*2.0;
t137 = L_fem.*dq1.*t15.*t45;
t138 = L_fem.*dq2.*t15.*t45;
t139 = L_fem.*dq5.*t15.*t45;
t148 = Lz_tib.*dq1.*t12.*t85;
t149 = Lz_tib.*dq2.*t12.*t85;
t150 = Lz_tib.*dq4.*t12.*t85;
t151 = Lz_tib.*dq5.*t12.*t85;
t152 = M_tib.*t11.*t45.*2.0;
t157 = Lz_tib.*t12.*t85.*2.0;
t158 = M_tib.*dq1.*t11.*t45;
t159 = M_tib.*dq2.*t11.*t45;
t160 = M_tib.*dq5.*t11.*t45;
t162 = L_fem.*dq2.*t39.*t45;
t165 = L_tib.*dq2.*t39.*t84;
t187 = L_tib.*dq1.*dq3.*t3.*t175;
t188 = g.*t31.*t176;
t189 = L_tib.*dq1.*t3.*t6.*t175;
t190 = L_tib.*dq3.*t3.*t17.*t175;
t97 = g.*t78;
t98 = g.*t79;
t99 = g.*t80;
t116 = -t73;
t117 = -t75;
t121 = g.*t92;
t122 = g.*t93;
t130 = g.*t119;
t131 = dq1.*t120;
t132 = dq5.*t120;
t134 = -t120;
t140 = L_tib.*dq1.*t125;
t141 = L_tib.*dq2.*t125;
t142 = L_tib.*dq3.*t125;
t143 = L_tib.*dq5.*t125;
t144 = L_tib.*dq1.*t126;
t145 = L_tib.*dq2.*t126;
t146 = L_tib.*dq3.*t126;
t147 = L_tib.*dq5.*t126;
t155 = L_tib.*t125.*2.0;
t156 = L_tib.*t126.*2.0;
t161 = L_tib.*t135;
t163 = -t157;
t166 = -t148;
t167 = -t149;
t168 = -t150;
t169 = -t151;
t170 = t16.*t135;
t174 = -t159;
t191 = -t188;
t192 = -t189;
t223 = t63+t65+t72+t78+t89+t92+t93+t111+t112+t113+t118+t119;
t226 = -g.*(t36-t63+t64-t65+t66+t67-t78+t79+t80-t89-t92-t93);
t129 = -t97;
t153 = -t131;
t154 = -t132;
t164 = -t141;
t171 = dq1.*t161;
t172 = dq3.*t161;
t173 = dq5.*t161;
t177 = t16.*t161.*2.0;
t179 = dq2.*t16.*t161;
t181 = dq4.*t16.*t161;
t183 = dq2.*t40.*t161;
t184 = dq4.*t40.*t161;
t193 = t134+t161;
t196 = dq4.*t18.*t40.*(t120-t161);
t201 = t125+t126+t170;
t178 = t16.*t171;
t180 = t16.*t172;
t182 = t16.*t173;
t186 = t18.*t181;
t194 = t19+t193;
t195 = t163+t177;
t198 = dq1.*(t157-t177).*(-1.0./2.0);
t199 = dq4.*(t157-t177).*(-1.0./2.0);
t200 = dq5.*(t157-t177).*(-1.0./2.0);
t204 = t155+t156+t177;
t205 = L_tib.*dq3.*t17.*t201;
t210 = t35+t68+t69+t70+t71+t201;
t212 = t37+t38+t153+t154+t171+t172+t173;
t221 = -dq1.*(t40.*t172+(dq1.*(t157-t177))./2.0+(dq5.*(t157-t177))./2.0);
t185 = t17.*t180;
t197 = dq4.*t16.*t194;
t203 = dq4.*t18.*t40.*t194;
t206 = -t205;
t207 = (dq2.*t204)./2.0;
t208 = (dq3.*t204)./2.0;
t209 = (dq5.*t204)./2.0;
t211 = L_tib.*dq3.*t17.*t210;
t213 = t136+t152+t155+t156+t195;
t214 = t49+t81+t82+t83+t114+t204;
t215 = dq5.*t16.*t212;
t220 = t180+t198+t200;
t229 = t57+t59+t96+t100+t102+t103+t105+t106+t108+t141+t143+t145+t147+t179+t181+t182;
t236 = t110+t137+t139+t140+t142+t143+t144+t146+t147+t158+t160+t166+t169+t178+t180+t182;
t243 = t52+t55+t62+t87+t96+t109+t138+t139+t141+t143+t145+t147+t159+t160+t167+t168+t169+t179+t181+t182;
t247 = t58+t61+t94+t95+t101+t104+t107+t110+t137+t140+t142+t144+t146+t149+t150+t158+t162+t164+t165+t166+t174+t178+t180+t183+t184;
t202 = t18.*t197;
t216 = (dq1.*t213)./2.0;
t217 = (dq2.*t213)./2.0;
t218 = (dq5.*t213)./2.0;
t219 = (dq3.*t214)./2.0;
t222 = t50+t114+t213;
t227 = t181+t207+t209;
t230 = dq5.*t229;
t237 = dq5.*t236;
t239 = t123+t130+t185+t215+t221;
t244 = dq5.*t243;
t248 = dq5.*t247;
t224 = (dq1.*t222)./2.0;
t225 = (dq5.*t222)./2.0;
t228 = dq2.*t227;
t231 = t197+t217+t218;
t233 = t199+t217+t218;
t238 = -t237;
t240 = t208+t216+t218;
t232 = dq2.*t231;
t235 = dq2.*t233;
t241 = dq1.*t240;
t245 = t219+t224+t225;
t249 = t186+t191+t192+t228+t230;
t234 = -t232;
t242 = -t241;
t246 = dq1.*t245;
t251 = t76+t98+t115+t116+t117+t127+t128+t129+t187+t190+t196+t235+t244;
t250 = t74+t77+t90+t99+t124+t206+t238+t242;
t252 = t203+t211+t226+t234+t246+t248;
bq = [-inv_D_mtx14.*t239+inv_D_mtx12.*t250+inv_D_mtx13.*t249+inv_D_mtx15.*(t202-t211+t232-t246-t248+g.*(t36-t63+t64-t65+t66+t67-t78+t79+t80-t89-t92-t93))-inv_D_mtx11.*(t73+t75+t91+t97+t121+t122-t187-t190-t235-t244+g.*t112+g.*t118+dq4.*t16.*t18.*(t120-t161));-inv_D_mtx24.*t239+inv_D_mtx22.*t250+inv_D_mtx23.*t249+inv_D_mtx25.*(t202-t211+t232-t246-t248+g.*(t36-t63+t64-t65+t66+t67-t78+t79+t80-t89-t92-t93))-inv_D_mtx21.*(t73+t75+t91+t97+t121+t122-t187-t190-t235-t244+g.*t112+g.*t118+dq4.*t16.*t18.*(t120-t161));-inv_D_mtx34.*t239+inv_D_mtx32.*t250+inv_D_mtx33.*t249+inv_D_mtx35.*(t202-t211+t232-t246-t248+g.*(t36-t63+t64-t65+t66+t67-t78+t79+t80-t89-t92-t93))-inv_D_mtx31.*(t73+t75+t91+t97+t121+t122-t187-t190-t235-t244+g.*t112+g.*t118+dq4.*t16.*t18.*(t120-t161));-inv_D_mtx44.*t239+inv_D_mtx42.*t250+inv_D_mtx43.*t249+inv_D_mtx45.*(t202-t211+t232-t246-t248+g.*(t36-t63+t64-t65+t66+t67-t78+t79+t80-t89-t92-t93))-inv_D_mtx41.*(t73+t75+t91+t97+t121+t122-t187-t190-t235-t244+g.*t112+g.*t118+dq4.*t16.*t18.*(t120-t161));-inv_D_mtx54.*t239+inv_D_mtx52.*t250+inv_D_mtx53.*t249+inv_D_mtx55.*(t202-t211+t232-t246-t248+g.*(t36-t63+t64-t65+t66+t67-t78+t79+t80-t89-t92-t93))-inv_D_mtx51.*(t73+t75+t91+t97+t121+t122-t187-t190-t235-t244+g.*t112+g.*t118+dq4.*t16.*t18.*(t120-t161))];