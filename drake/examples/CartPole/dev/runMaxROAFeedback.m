function runMaxROAFeedback

p = CartPolePlant;
p = p.setInputLimits(-inf,inf);

x0=[0;pi;0;0]; u0=0;
pp = p.taylorApprox(0,x0,0,3);  % make polynomial approximation

options = struct();
options.degL = 2;
%options.degK = 3;
[K,V] = maxROAFeedback(pp,x0,u0,options);

% NOTEST
