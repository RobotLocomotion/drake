function testNLPWConstraint
nlp = NonlinearProgramWConstraint(2);
cnstr1 = NonlinearConstraint(-inf(2,1),[4;5],[0;-inf],inf(2,1),@cnstr1_userfun);
nlp = nlp.addNonlinearConstraint(cnstr1);
x1 = [1;2];
[g1,h1,dg1,dh1] = nlp.nonlinearConstraints(x1);
[g1_user,dg1_user] = cnstr1_userfun(x1);
valuecheck(g1,g1_user);
valuecheck(dg1,dg1_user);
if(~isempty(h1) || ~isempty(dh1))
  error('There should be no nonlinear equality constraint');
end
keyboard;
end

function [c,dc] = cnstr1_userfun(x)
c = [x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2];
dc = [2*x(1) 8*x(2);2*(x(1)-2) 2*x(2)];
end