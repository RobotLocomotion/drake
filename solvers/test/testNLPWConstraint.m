function testNLPWConstraint
nlp1 = NonlinearProgramWConstraintObjects(3);
nlp1 = nlp1.setCheckGrad(true);
warning('Off','optimlib:fmincon:WillRunDiffAlg');
warning('Off','optimlib:fmincon:SwitchingToMediumScale');
%%%%%%%%%%%
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
cnstr1 = NonlinearConstraint(-inf(2,1),[4;5],2,@cnstr1_userfun);
xind1 = [1;2];
nlp1 = nlp1.addNonlinearConstraint(cnstr1,xind1);
nlp1 = nlp1.addBoundingBoxConstraint(BoundingBoxConstraint(0,inf),1);
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
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
A = [1 0 2;1 0 3];
nlp1 = nlp1.addLinearConstraint(LinearConstraint([0;0],[10;0],[1 2;1 3]),[1;3]);
valuecheck(nlp1.beq,0);
valuecheck(nlp1.Aeq,[1 0 3]);
valuecheck(nlp1.Ain,[A(1,:);-A(1,:)]);
valuecheck(nlp1.bin,[10;0]);
x0 = [1;2;4];
[x1,F,info] = nlp1.solve(x0);
if(info>10)
  error('SNOPT fails');
end
c1 = cnstr1_userfun(x1);
if(c1(1)>4+1e-5 || c1(2)>5+1e-5)
  error('Wrong transcription for SNOPT nonlinear constraint');
end
b1 = A*x1;
if(b1(1)>1+1e-5 || b1(1)<0-1e-5 || abs(b1(2))>1e-5)
  error('Wrong transcription for SNOPT linear constraint');
end
if(x1(1)<-1e-5)
  error('Wrong transcription for SNOPT x_lb');
end
nlp2 = nlp1.setSolver('fmincon');
[x1,F,info] = nlp2.solve(x0);
c1 = cnstr1_userfun(x1);
if(c1(1)>4+1e-5 || c1(2)>5+1e-5)
  error('Wrong transcription for SNOPT nonlinear constraint');
end
b1 = A*x1;
if(b1(1)>1+1e-5 || b1(1)<0-1e-5 || abs(b1(2))>1e-5)
  error('Wrong transcription for SNOPT linear constraint');
end
if(x1(1)<-1e-5)
  error('Wrong transcription for SNOPT x_lb');
end
%%%%%%%%%%%%%%%%
% min x2^2+x1*x3+x3
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
nlp1 = nlp1.addCost(NonlinearConstraint(-inf,inf,1,@cost1_userfun),2);
nlp1 = nlp1.addCost(NonlinearConstraint(-inf,inf,2,@cost2_userfun),[1;3]);
nlp1 = nlp1.addCost(LinearConstraint(-inf,inf,1),3);
[x2,F,info] = nlp1.solve(x0);
if(info>10)
  error('SNOPT fails');
end
c2 = cnstr1_userfun(x2);
if(c2(1)>4+1e-5 || c2(2)>5+1e-5)
  error('Wrong transcription for SNOPT nonlinear constraint');
end
b2 = A*x2;
if(b2(1)>1+1e-5 || b2(1)<0-1e-5 || abs(b2(2))>1e-5)
  error('Wrong transcription for SNOPT linear constraint');
end
if(x2(1)<-1e-5)
  error('Wrong transcription for x_lb');
end
f2 = cost1_userfun(x2(2))+cost2_userfun(x2([1;3]))+x2(3);
valuecheck(F,f2);
nlp2 = nlp1.setSolver('fmincon');
[x2_fmincon,F,info] = nlp2.solve(x0);
valuecheck(x2,x2_fmincon,1e-4);

%%%%%%%%%%%%%%%%%%%%%
% min x2^2+x1*x3+x3
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
% x1*x2*x3 = 1/6;
% -10<=x2^2+x2*x3+2*x3^2<=30
nc2 = NonlinearConstraint([1/6;-10],[1/6;30],3,@cnstr2_userfun);
nc2 = nc2.setSparseStructure([1;1;1;2;2],[1;2;3;2;3]);
nlp1 = nlp1.addNonlinearConstraint(nc2);
x0 = [1;2;3];
[x,F,info] = nlp1.solve(x0);
c2 = cnstr2_userfun(x);
valuecheck(c2(1),1/6,1e-5);
if(info>10)
  error('SNOPT fails');
end
nlp2 = nlp1.setSolver('fmincon');
[x_fmincon,F,info] = nlp2.solve(x0);
valuecheck(x,x_fmincon,1e-4);
% min x2^2+x1*x3+x3
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
% x1*x2*x3 = 1/6;
% -10<=x2^2+x2*x3+2*x3^2<=30
% x2+x3<=10
% -x2+x3 = 0.1;
nlp1 = nlp1.addLinearEqualityConstraints([0 -1 1],0.1);
nlp1 = nlp1.addLinearInequalityConstraints([0 1 1],10);
valuecheck(nlp1.bin,[10;0;10]);
valuecheck(nlp1.beq,[0;0.1]);
valuecheck(nlp1.Ain,[A(1,:);-A(1,:);0 1 1]);
valuecheck(nlp1.Aeq,[1 0 3;0 -1 1]);
[x,F,info] = nlp1.solve(x0);
if(info>10)
  error('SNOPT fails');
end
valuecheck(-x(2)+x(3),0.1,1e-5);
if(x(2)+x(3)>10+1e-5)
  error('Linear constraint not correct');
end
nlp2 = nlp1.setSolver('fmincon');
[x_fmincon,F,info] = nlp2.solve(x0);
valuecheck(x,x_fmincon,1e-4);

display('test addDecisionVar');
nlp1 = nlp1.addDecisionVariable(2);
[x,F,info] = nlp1.compareSolvers([x0;randn(2,1)]);

display('test replaceCost');
% min x2^2+x1^2+x1x2+x1*x3+x3
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
% x1*x2*x3 = 1/6;
% -10<=x2^2+x2*x3+2*x3^2<=30
% x2+x3<=10
% -x2+x3 = 0.1;
qc1 = QuadraticConstraint(-inf,inf,[1 0.5;0.5 1],zeros(2,1));
nlp1 = nlp1.replaceCost(qc1,1,[1;2]);
[x,F,info] = nlp1.compareSolvers([x0;randn(2,1)]);
F_obj = qc1.eval(x{1}(1:2));
F_obj = F_obj+cost2_userfun(x{1}([1;3]))+x{1}(3);
valuecheck(F{1},F_obj,1e-5);
end

function [c,dc] = cnstr1_userfun(x)
c = [x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2];
dc = [2*x(1) 8*x(2);2*(x(1)-2) 2*x(2)];
end

function [c,dc] = cost1_userfun(x)
c = x^2;
dc = 2*x;
end

function [c,dc] = cost2_userfun(x)
c = x(1)*x(2);
dc = [x(2) x(1)];
end

function [c,dc] = cnstr2_userfun(x)
c = [x(1)*x(2)*x(3);x(2)^2+x(2)*x(3)+2*x(3)^2];
dc = [x(2)*x(3) x(1)*x(3) x(1)*x(2); 0 2*x(2)+x(3) x(2)+4*x(3)];
end