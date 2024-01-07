function symbolic_y_ddot = func_y_ddot(alpha,alpha_dot,beta,beta_dot,u1,u2)
%FUNC_Y_DDOT
%    SYMBOLIC_Y_DDOT = FUNC_Y_DDOT(ALPHA,ALPHA_DOT,BETA,BETA_DOT,U1,U2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    07-Jan-2024 11:04:46

t2 = cos(beta);
t3 = sin(alpha);
t4 = sin(beta);
t5 = alpha_dot.^2;
t6 = beta.*2.0;
t7 = beta_dot.^2;
t8 = cos(t6);
t9 = sin(t6);
t10 = alpha+t6;
t15 = beta_dot.*t2.*6.768715453849653e+26;
t17 = t2.*u2.*1.741372640558182e+32;
t11 = sin(t10);
t12 = t8.*4.56261632e+8;
t16 = -t15;
t18 = t5.*t9.*8.416541556157048e+28;
t19 = -t17;
t13 = t12-1.64084533245e+11;
t14 = 1.0./t13;
et1 = alpha_dot.*3.383003808543871e+27-beta_dot.*3.421737133440028e+27-t3.*2.508038558387371e+32+t11.*5.16039204161879e+30+t16+t18+t19+u1.*8.803028385490165e+32;
et2 = u2.*-8.803028385490165e+32+t4.*t5.*8.509500207004381e+29+t4.*t7.*8.509500207004381e+29+alpha_dot.*beta_dot.*t4.*1.701900041400876e+30;
et3 = alpha_dot.*1.691501904271935e+27-beta_dot.*1.378248762589505e+28-t3.*1.254019279193685e+32+t11.*2.580196020809395e+30+t16+t18+t19+u1.*4.401514192745083e+32;
et4 = u2.*-3.54579048775278e+33+sin(alpha+beta).*1.714069110394488e+32-sin(alpha-beta).*1.265840341247194e+31+alpha_dot.*t2.*3.346047528832546e+26+t4.*t5.*3.427559649729151e+30;
et5 = t4.*t7.*4.254750103502191e+29+t7.*t9.*4.208270778078524e+28+t2.*u1.*8.706863202790908e+31+alpha_dot.*beta_dot.*t4.*8.509500207004381e+29+alpha_dot.*beta_dot.*t9.*8.416541556157048e+28;
symbolic_y_ddot = [t14.*(et1+et2).*(-5.421010862427522e-21);(t14.*(et3+et4+et5))./9.223372036854776e+19];
end
