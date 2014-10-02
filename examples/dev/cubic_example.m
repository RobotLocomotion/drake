
% Define a new symbolic variable
x=msspoly('x');

% Define the dynamics and Lyapunov function
f = -x + x^3;
V = x^2;
Vdot = diff(V,x)*f;

% construct a SOS program
prog=mssprog;

% construct the decision variables
[prog,c] = new(prog,length(m),'free');
[prog,slack] = new(prog,1,'free');

% construct the langrange multiplier
m=monomials(x,0:4);
lambda = c'*m;

% add the SOS constraints
prog.sos=-(Vdot+lambda*(1-V))-slack*x^2;
prog.sos=lambda;

% solve the problem using -slack as the objective
prog.sedumi=-slack;  

% display the results
slack=double(prog(slack))
c = double(prog(c))



% Everything after this is just plotting...



%c = [0 0 1.6375 0 .3625]';
xs=-1.2:.05:1.2;
clear lambda vl;
for i=1:length(xs)
  lambda(i)=double(subs(c'*m,x,xs(i)));
  vl(i)=double(subs(-2*x^2+2*x^4+(c'*m)*(1-x^2),x,xs(i)));
end

clf;
hold on;
plot(xs,-xs+xs.^3,'k','linewidth',2);
legend('xdot');
axis tight
pause;

plot(xs,-2*xs.^2+2*xs.^4,'r','linewidth',2);
legend('xdot','Vdot');
pause;

plot(xs,lambda,'g','linewidth',2);
legend('xdot','Vdot','lambda');
pause;

plot(xs,vl,'b','linewidth',2);
legend('xdot','Vdot','lambda','Vdot+lambda*(1-V)');

