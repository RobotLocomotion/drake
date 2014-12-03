ydotdot= 0:0.1:7;
a = 1;
C=20/exp(6.5);
b=0;
z = b + C*exp(a*ydotdot);
plot(ydotdot,z)