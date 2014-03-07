function testNLPWConstraint
nlp1 = NonlinearProgramWConstraint(3);
%%%%%%%%%%%
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
cnstr1 = NonlinearConstraint(-inf(2,1),[4;5],[0;-inf],inf(2,1),@cnstr1_userfun);
xind1 = [1;2];
nlp1 = nlp1.addNonlinearConstraint(cnstr1,xind1);
x1 = [1;2;1];
[g1,h1,dg1,dh1] = nlp1.nonlinearConstraints(x1);
[g1_user,dg1_user] = cnstr1_userfun(x1(xind1));
valuecheck(g1,g1_user);
valuecheck(dg1(:,xind1),dg1_user);
if(~isempty(h1) || ~isempty(dh1))
  error('There should be no nonlinear equality constraint');
end
%%%%%%%%%
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+x3 <=1
% x1+3*x3 = 0
nlp1 = nlp1.addLinearConstraint(LinearConstraint([0;0],[1;0],-inf(2,1),inf(2,1),[1 1;1 3]),[1;3]);
valuecheck(nlp1.bin_lb,0);
valuecheck(nlp1.bin_ub,1);
valuecheck(nlp1.beq,0);
valuecheck(nlp1.Aeq,[1 0 3]);
valuecheck(nlp1.Ain,[1 0 1]);
keyboard;
end

function [c,dc] = cnstr1_userfun(x)
c = [x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2];
dc = [2*x(1) 8*x(2);2*(x(1)-2) 2*x(2)];
end