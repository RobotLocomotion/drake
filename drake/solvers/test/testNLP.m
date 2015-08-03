function testNLP
nlp0 = NonlinearProgram(0);
sol = nlp0.solve([]);
if(~isempty(sol))
  error('Solution should be empty for this problem');
end
nlp1 = NonlinearProgram(3);
nlp1 = nlp1.setCheckGrad(true);
warning('Off','optimlib:fmincon:WillRunDiffAlg');
warning('Off','optimlib:fmincon:SwitchingToMediumScale');
%%%%%%%%%%%
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
cnstr1 = FunctionHandleConstraint(-inf(2,1),[4;5],2,@cnstr1_userfun);
xind1 = [1;2];
nlp1 = nlp1.addConstraint(cnstr1,xind1);
nlp1 = nlp1.addConstraint(BoundingBoxConstraint(0,inf),1);
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
nlp1 = nlp1.setSolverOptions('snopt','print','print.out');
[x1,F,info] = testAllSolvers(nlp1,x0);
c1 = cnstr1_userfun(x1);
if(c1(1)>4+1e-5 || c1(2)>5+1e-5)
  error('Wrong transcription nonlinear constraint');
end
b1 = A*x1;
if(b1(1)>1+1e-5 || b1(1)<0-1e-5 || abs(b1(2))>1e-5)
  error('Wrong transcription for linear constraint');
end
if(x1(1)<-1e-5)
  error('Wrong transcription for x_lb');
end
%%%%%%%%%%%%%%%%
% min x2^2+x1*x3+x3
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
nlp1 = nlp1.addCost(FunctionHandleObjective(1,@cost1_userfun),2);
nlp1 = nlp1.addCost(FunctionHandleObjective(2,@cost2_userfun),[1;3]);
nlp1 = nlp1.addCost(LinearConstraint(-inf,inf,[0 1]),[1;3]);
[x2,F,info] = testAllSolvers(nlp1,x0);
c2 = cnstr1_userfun(x2);
if(c2(1)>4+1e-5 || c2(2)>5+1e-5)
  error('Wrong transcription for nonlinear constraint');
end
b2 = A*x2;
if(b2(1)>1+1e-5 || b2(1)<0-1e-5 || abs(b2(2))>1e-5)
  error('Wrong transcription for linear constraint');
end
if(x2(1)<-1e-5)
  error('Wrong transcription for x_lb');
end
f2 = cost1_userfun(x2(2))+cost2_userfun(x2([1;3]))+x2(3);
valuecheck(F,f2);
%%%%%%%%%%%%%%%%%%%%%
% min x2^2+x1*x3+x3
% x1^2+4*x2^2<=4
% (x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
% x1*x2*x3 = 1/6;
% -10<=x2^2+x2*x3+2*x3^2<=30
nc2 = FunctionHandleConstraint([1/6;-10],[1/6;30],3,@cnstr2_userfun);
nc2 = nc2.setSparseStructure([1;1;1;2;2],[1;2;3;2;3]);
nlp1 = nlp1.addConstraint(nc2);
x0 = [1;2;3];
[x,F,info] = testAllSolvers(nlp1,x0);
c2 = cnstr2_userfun(x);
valuecheck(c2(1),1/6,1e-5);
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
nlp1 = nlp1.addLinearConstraint(LinearConstraint(0.1,0.1,[0 -1 1]));
nlp1 = nlp1.addLinearConstraint(LinearConstraint(-inf,10,[0 1 1]));
valuecheck(nlp1.bin,[10;0;10]);
valuecheck(nlp1.beq,[0;0.1]);
valuecheck(nlp1.Ain,[A(1,:);-A(1,:);0 1 1]);
valuecheck(nlp1.Aeq,[1 0 3;0 -1 1]);
[x,F,info] = testAllSolvers(nlp1,x0);
valuecheck(-x(2)+x(3),0.1,1e-5);
if(x(2)+x(3)>10+1e-5)
  error('Linear constraint not correct');
end

display('test addDecisionVar');
nlp1 = nlp1.addDecisionVariable(2);
[x,F,info] = testAllSolvers(nlp1,[x0;randn(2,1)]);

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
[x,F,info] = testAllSolvers(nlp1,[x0;randn(2,1)]);
F_obj = qc1.eval(x(1:2));
F_obj = F_obj+cost2_userfun(x([1;3]))+x(3);
valuecheck(F,F_obj,1e-5);

%%%%%%%%%%%%%%%%%%%%%
display('test multi-input constraint and cost')
% min x1*x2
% -10<=x1^2+4*x2^2<=4
% -10<=(x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
nlp3 = NonlinearProgram(3);
nlp3 = nlp3.setCheckGrad(true);
cnstr3 = FunctionHandleConstraint([-10;-10],[4;5],2,@cnstr3_userfun);
cost3 = FunctionHandleObjective(2,@cost3_userfun);
xind1 = {1;2};
[nlp3,cnstr3_id] = nlp3.addConstraint(cnstr3,xind1);
cnstr3_id_old = cnstr3_id;
nlp3 = nlp3.addCost(cost3,xind1);
bbcon1 = BoundingBoxConstraint(0,inf);
[nlp3,bbcon1_id] = nlp3.addBoundingBoxConstraint(bbcon1,1);

A = [1 0 2;1 0 3];
lincon1 = LinearConstraint([0;0],[10;0],[1 2;1 3]);
[nlp3,lincon1_id] = nlp3.addLinearConstraint(lincon1,[1;3]);
x0 = [1;2;4];
[x3,F,info] =testAllSolvers(nlp3,x0);
c3 = cnstr3_userfun(x3(1),x3(2));
if(c3(1)>4+1e-5 || c3(2)>5+1e-5)
  error('Wrong transcription for nonlinear constraint');
end
b1 = A*x3;
if(b1(1)>1+1e-5 || b1(1)<0-1e-5 || abs(b1(2))>1e-5)
  error('Wrong transcription for linear constraint');
end
if(x3(1)<-1e-5)
  error('Wrong transcription for x_lb');
end

%%%%%%%%%%%%%%%%%%%
display('test deleteNonlinearConstraint')
display('first try to delete all nonlinear constraint');
nlp4 = nlp3.deleteNonlinearConstraint(cnstr3_id);
if(nlp4.isNonlinearConstraintID(cnstr3_id))
  error('The deleted ID should not be contained in the program any more');
end
valuecheck(nlp4.num_cin,0);
valuecheck(nlp4.num_ceq,0);
if(~isempty(nlp4.cin_lb) || ~isempty(nlp4.cin_ub) )
  error('The lower and upper bound of cin should be empty');
end
[lb,ub] = nlp4.bounds();
valuecheck(lb,[-inf;-inf;-inf;0]);
valuecheck(ub,[inf;10;0;0]);
[iGfun,jGvar] = nlp4.getNonlinearGradientSparsity();
valuecheck([1 1 1],full(sparse(iGfun,jGvar,ones(size(iGfun)),1+nlp4.num_cin+nlp4.num_ceq,nlp4.num_vars)));
[g,h,dg,dh] = nlp4.nonlinearConstraints(x0);
if(~isempty(g) || ~isempty(h) || ~isempty(dg) || ~isempty(dh))
  error('The nonlinear constraint should be empty');
end
[x4,F,info] = testAllSolvers(nlp4,x0);

% min x1*x2
% -10<=x1^2+4*x2^2<=4
% -10<=(x1-2)^2+x2^2<=5
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
% x1*x2+x3 = 0
% 0<=x1+x3^2<=10
% x1+2*x3*x4+x4^2 = 5
[nlp4,cnstr3_id] = nlp4.addConstraint(cnstr3,xind1);
nlp4 = nlp4.addDecisionVariable(1);
cnstr4 = FunctionHandleConstraint([0;0;5],[0;10;5],4,@cnstr4_userfun,2);
[nlp4,cnstr4_id] = nlp4.addConstraint(cnstr4);
[x4,F,info] = testAllSolvers(nlp4,[x0;0]);

nlp4 = nlp4.deleteNonlinearConstraint(cnstr3_id);
if(nlp4.isNonlinearConstraintID(cnstr3_id))
  error('The deleted ID should not be contained in the program any more');
end
valuecheck(nlp4.num_cin,1);
valuecheck(nlp4.num_ceq,2);
valuecheck(nlp4.cin_lb,0);
valuecheck(nlp4.cin_ub,10);
valuecheck(length(nlp4.nlcon),1);
sizecheck(nlp4.cin_name,1);
sizecheck(nlp4.ceq_name,2);
sizecheck(nlp4.nlcon_xind,1);
valuecheck(nlp4.nlcon_xind{1}{1},[1;2;3;4]);
[iGfun,jGvar] = nlp4.getNonlinearGradientSparsity();
valuecheck([1 1 1 0;ones(3,4)],full(sparse(iGfun,jGvar,ones(size(iGfun)),1+nlp4.num_cin+nlp4.num_ceq,nlp4.num_vars)));
[g,h,dg,dh] = nlp4.nonlinearConstraints([x0;1]);
[c4,dc4] = cnstr4_userfun([x0;1]);
valuecheck(g,c4(2));
valuecheck(h,c4([1;3])-[0;5]);
valuecheck(dg,dc4(2,:));
valuecheck(dh,dc4([1;3],:));
[x4,F,info] = testAllSolvers(nlp4,x4);

nlp4 = nlp4.deleteNonlinearConstraint(cnstr4_id);
if(nlp4.isNonlinearConstraintID(cnstr4_id))
  error('The deleted ID should not be contained in the program any more');
end
valuecheck(nlp4.num_cin,0);
valuecheck(nlp4.num_ceq,0);
if(~isempty(nlp4.cin_lb) || ~isempty(nlp4.cin_ub) )
  error('The lower and upper bound of cin should be empty');
end
[lb,ub] = nlp4.bounds();
valuecheck(lb,[-inf;-inf;-inf;0]);
valuecheck(ub,[inf;10;0;0]);
[iGfun,jGvar] = nlp4.getNonlinearGradientSparsity();
valuecheck([1 1 1 0],full(sparse(iGfun,jGvar,ones(size(iGfun)),1+nlp4.num_cin+nlp4.num_ceq,nlp4.num_vars)));
[g,h,dg,dh] = nlp4.nonlinearConstraints(x0);
if(~isempty(g) || ~isempty(h) || ~isempty(dg) || ~isempty(dh))
  error('The nonlinear constraint should be empty');
end
[x4,F,info] = testAllSolvers(nlp4,[x0;1]);

%%%%%%
display('check updateNonlinearConstraint');
nlp4 = nlp3.addDecisionVariable(1);
[nlp4,new_cnstr_id] = nlp4.updateNonlinearConstraint(cnstr3_id_old,cnstr4);
valuecheck(nlp4.num_cin,1);
valuecheck(nlp4.num_ceq,2);
valuecheck(nlp4.cin_lb,0);
valuecheck(nlp4.cin_ub,10);
valuecheck(length(nlp4.nlcon),1);
sizecheck(nlp4.cin_name,1);
sizecheck(nlp4.ceq_name,2);
sizecheck(nlp4.nlcon_xind,1);
valuecheck(nlp4.nlcon_xind{1}{1},[1;2;3;4]);
[iGfun,jGvar] = nlp4.getNonlinearGradientSparsity();
valuecheck([1 1 1 0;ones(3,4)],full(sparse(iGfun,jGvar,ones(size(iGfun)),1+nlp4.num_cin+nlp4.num_ceq,nlp4.num_vars)));
[g,h,dg,dh] = nlp4.nonlinearConstraints([x0;1]);
[c4,dc4] = cnstr4_userfun([x0;1]);
valuecheck(g,c4(2));
valuecheck(h,c4([1;3])-[0;5]);
valuecheck(dg,dc4(2,:));
valuecheck(dh,dc4([1;3],:));
[x4,F,info] = testAllSolvers(nlp4,x4);
[~,new_cnstr_idx] = nlp4.isNonlinearConstraintID(new_cnstr_id);
valuecheck(nlp4.nlcon{new_cnstr_idx}.eval(x4),cnstr4.eval(x4));

%%%%%
display('check deleteLinearConstraint')
nlp4 = nlp4.deleteLinearConstraint(lincon1_id);
if(~isempty(nlp4.Ain) || ~isempty(nlp4.bin) || ~isempty(nlp4.Aeq) || ~isempty(nlp4.beq) || ~isempty(nlp4.Ain_name) || ~isempty(nlp4.Aeq_name) || ~isempty(nlp4.lcon))
  error('The linear constraint should be empty');
end
if(nlp4.isLinearConstraintID(lincon1_id))
  error('The deleted ID should not be contained in the program any more');
end
[x4,F,info] = testAllSolvers(nlp4,x4);

[nlp4,lincon1_id] = nlp4.addLinearConstraint(lincon1,[1;3]);
sizecheck(nlp4.Ain,[2,4]);
sizecheck(nlp4.Aeq,[1,4]);
sizecheck(nlp4.bin,[2,1]);
sizecheck(nlp4.beq,[1,1]);
[x4,F,info] = testAllSolvers(nlp4,x4);
if(x4(1)<0 || x4(1)+2*x4(3)>10+1e-6 || x4(1)+2*x4(3)<0-1e-6 || abs(x4(1)+3*x4(3))>1e-6)
  error('linear constraint is not satisfied');
end
% min x1*x2
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
% x1*x2+x3 = 0
% 0<=x1+x3^2<=10
% x1+2*x3*x4+x4^2 = 5
% x1+x4<=5
% x2+x3 = 2
% -5<=x1+2*x3+4*x4<=10
lincon2 = LinearConstraint([-inf;2;-5],[5;2;10],[1 0 0 1;0 1 1 0;1 0 2 4]);
[nlp4,lincon2_id] = nlp4.addLinearConstraint(lincon2);
[x4,F,info] = testAllSolvers(nlp4,x4);

nlp4 = nlp4.deleteLinearConstraint(lincon1_id);
sizecheck(nlp4.Ain,[3,4]);
sizecheck(nlp4.bin,[3,1]);
sizecheck(nlp4.Aeq,[1,4]);
sizecheck(nlp4.beq,[1,1]);
if(nlp4.isLinearConstraintID(lincon1_id))
  error('The deleted linear constraint should not be contained in the program');
end
[x4,F,info] = testAllSolvers(nlp4,x4);
if(any(lincon2.eval(x4)>lincon2.ub+1e-5) || any(lincon2.eval(x4)<lincon2.lb-1e-5))
  error('Linear constraint is not satisfied');
end

nlp4 = nlp4.deleteLinearConstraint(lincon2_id);
if(~isempty(nlp4.Ain) || ~isempty(nlp4.bin) || ~isempty(nlp4.Aeq) || ~isempty(nlp4.beq) || ~isempty(nlp4.Ain_name) || ~isempty(nlp4.Aeq_name) || ~isempty(nlp4.lcon))
  error('The linear constraint should be empty');
end
if(nlp4.isLinearConstraintID(lincon2_id))
  error('The deleted ID should not be contained in the program any more');
end
[x4,F,info] = testAllSolvers(nlp4,x4);

[nlp4,lincon2_id] = nlp4.addLinearConstraint(lincon2);
nlp4 = nlp4.updateLinearConstraint(lincon2_id,lincon1,[1;3]);
sizecheck(nlp4.Ain,[2,4]);
sizecheck(nlp4.bin,[2,1]);
sizecheck(nlp4.Aeq,[1,4]);
sizecheck(nlp4.beq,[1,1]);
valuecheck(length(nlp4.lcon),1);
[x4,F,info] = testAllSolvers(nlp4,x4);
if(any(lincon1.eval(x4([1;3]))>lincon1.ub+1e-5) || any(lincon1.eval(x4([1;3]))<lincon1.lb-1e-5))
  error('Linear constraint not satisfied');
end

%%%
display('test deleteBoundingBoxConstraint');
nlp4 = nlp4.deleteBoundingBoxConstraint(bbcon1_id);
if(any(~isinf(nlp4.x_lb)) || any(~isinf(nlp4.x_ub)) || ~isempty(nlp4.bbcon) || ~isempty(nlp4.bbcon_xind))
  error('The bounding box constraint should be empty');
end
if(nlp4.isBoundingBoxConstraintID(bbcon1_id))
  error('The deleted constraint should not be contained in the program');
end
[x4,F,info] = testAllSolvers(nlp4,x4);


[nlp4,bbcon1_id] = nlp4.addBoundingBoxConstraint(bbcon1,1);
valuecheck(nlp4.bbcon_xind{1},1);

% min x1*x2
% x1 >= 0
% 0<=x1+2*x3 <=10
% x1+3*x3 = 0
% x1*x2+x3 = 0
% 0<=x1+x3^2<=10
% x1+2*x3*x4+x4^2 = 5
% -1<= x2 <= 10
%  x1 = 0;
bbcon2 = BoundingBoxConstraint([0;-1],[0;10]);
[nlp4,bbcon2_id] = nlp4.addBoundingBoxConstraint(bbcon2,[1;2]);
valuecheck(nlp4.x_lb,[0;-1;-inf;-inf]);
valuecheck(nlp4.x_ub,[0;10;inf;inf]);
valuecheck(nlp4.bbcon_xind{2},[1;2]);
[x4,F,info] = testAllSolvers(nlp4,x4);

nlp4 = nlp4.deleteBoundingBoxConstraint(bbcon2_id);
valuecheck(nlp4.x_lb,[0;-inf;-inf;-inf]);
valuecheck(nlp4.x_ub,inf(4,1));
[nlp4,bbcon2_id] = nlp4.addBoundingBoxConstraint(bbcon2,[1;2]);
nlp4 = nlp4.deleteBoundingBoxConstraint(bbcon1_id);
valuecheck(nlp4.x_lb,[0;-1;-inf;-inf]);
valuecheck(nlp4.x_ub,[0;10;inf;inf]);
valuecheck(nlp4.bbcon_xind{1},[1;2]);
sizecheck(nlp4.bbcon_xind,[1,1]);
sizecheck(nlp4.bbcon,[1,1]);

[nlp4,bbcon1_id] = nlp4.updateBoundingBoxConstraint(bbcon2_id,bbcon1,1);
valuecheck(nlp4.x_lb,[0;-inf;-inf;-inf]);
valuecheck(nlp4.x_ub,inf(4,1));
sizecheck(nlp4.bbcon,[1,1]);
sizecheck(nlp4.bbcon_xind,[1,1]);


% check add empty linear constraint
[nlp4,null_lincon_id] = nlp4.addLinearConstraint(LinearConstraint([],[],zeros(0,nlp4.num_vars)));
nlp4 = nlp4.deleteLinearConstraint(null_lincon_id);
end

function [c,dc] = cnstr1_userfun(x)
c = [x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2];
if(nargout>1)
dc = [2*x(1) 8*x(2);2*(x(1)-2) 2*x(2)];
end
end

function [c,dc] = cnstr3_userfun(x1,x2)
c = [x1^2+4*x2^2;(x1-2)^2+x2^2];
if(nargout>1)
dc = [2*x1 8*x2;2*(x1-2) 2*x2];
end
end

function [c,dc] = cost1_userfun(x)
c = x^2;
if(nargout>1)
dc = 2*x;
end
end

function [c,dc] = cost2_userfun(x)
c = x(1)*x(2);
if(nargout>1)
dc = [x(2) x(1)];
end
end

function [c,dc] = cost3_userfun(x1,x2)
c = x1*x2;
if(nargout>1)
dc = [x2 x1];
end
end

function [c,dc] = cost4_userfun(x)
c = x(1)+x(2)^2;
dc = [1 2*x(2)];
end

function [c,dc] = cnstr2_userfun(x)
c = [x(1)*x(2)*x(3);x(2)^2+x(2)*x(3)+2*x(3)^2];
if(nargout>1)
dc = [x(2)*x(3) x(1)*x(3) x(1)*x(2); 0 2*x(2)+x(3) x(2)+4*x(3)];
end
end

function [c,dc] = cnstr4_userfun(x)
c = [x(1)*x(2)+x(3);x(1)+x(3)^2;x(1)+2*x(3)*x(4)+x(4)^2];
if(nargout>1)
  dc = [x(2) x(1) 1 0;1 0 2*x(3) 0;1 0 2*x(4) 2*x(3)+2*x(4)];
end
end

function [c,dc] = cnstr5_userfun(x)
c = x(1)+x(2)+x(3)*x(2);
dc = [1 1+x(3) x(2)];
end

  function [x,F,info] = solveWDefaultSolver(nlp,x)
    nlp = nlp.setSolver('default');
    if(strcmp(nlp.solver,'fmincon'))
      nlp = nlp.setSolverOptions('fmincon','Algorithm','sqp');
      nlp = nlp.setSolverOptions('fmincon','MaxIter',1000);
    end
    [x,F,info] = nlp.solve(x);
    if(info ~= 1)
      error('failed to solve the problem');
    end
  end
  
  function [x,F,info] = testAllSolvers(nlp,x)
    nlp = nlp.setSolverOptions('fmincon','Algorithm','sqp');
    nlp = nlp.setSolverOptions('fmincon','MaxIter',1000);
    [x,F,info,execution_time,solvers] = nlp.compareSolvers(x);
    num_solvers = length(x);
    for i = 1:num_solvers
      if(info{i}~= 1)
        error('The problem is not solved with %s',solvers{i});
      end
      [g,h] = nlp.nonlinearConstraints(x{i});
      if any(g<nlp.cin_lb-1e-3 | g>nlp.cin_ub+1e-3)
        error('The nonlinear inequality constraint is not transcribed correctly with %s',solvers{i});
      end
      if any(abs(h)>1e-4)
        error('The nonlinear equality constraint is not transcribed correctly with %s', solvers{i});
      end
      if ~isempty(nlp.Ain)
        lin_ineq = nlp.Ain*x{i};
        if(any(lin_ineq>nlp.bin+1e-3))
          error('The linear inequality constraint is not transcribed correctly with %s',solvers{i});
        end
      end
      if(~isempty(nlp.Aeq))
        lin_eq = nlp.Aeq*x{i};
        if(any(abs(lin_eq-nlp.beq)>1e-3))
          error('The linear equality constraint is not transcribed correctly with %s',solvers{i});
        end
      end
      if(any(x{i}-nlp.x_ub>1e-4) || any(x{i}-nlp.x_lb<-1e-4))
        error('The bounding box constraint on x is not transcribed correctly with %s',solvers{i});
      end
    end
    x = x{1};
    F = F{1};
    info = info{1};
  end
  
  