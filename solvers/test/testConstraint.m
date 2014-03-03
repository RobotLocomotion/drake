function testConstraint
display('Check constructing constraint with anonymous function handle without gradient');
cnstr1 = Constraint(0,1,[],[],@(x) x+1);
[c1,dc1] = cnstr1.eval(1);
valuecheck(cnstr1.relation,Constraint.ineq);
valuecheck(c1,2);
valuecheck(dc1,1);
[lb,ub] = cnstr1.bounds();
valuecheck(lb,0);
valuecheck(ub,1);

display('Check constructing constraint with function handle and 1st order gradient');
cnstr2 = Constraint([0;1],[0;2],[],[],@cnstr_userfun2);
[c2,dc2] = cnstr2.eval([1;2]);
[c2_user,dc2_user] = geval(@cnstr_userfun2,[1;2],struct('grad_method','taylorvar'));
valuecheck(c2,c2_user);
valuecheck(dc2,dc2_user);
valuecheck(cnstr2.relation,[Constraint.equal;Constraint.ineq]);

display('Check constructing constraint with function handle and 2nd order gradient');
cnstr3 = Constraint([0;1],[0;2],[],[],@cnstr_userfun3);
[c3,dc3,ddc3] = cnstr3.eval([1;2]);
[c3_user,dc3_user,ddc3_user] = cnstr_userfun3([1;2]);
valuecheck(c3,c3_user);
valuecheck(dc3,dc3_user);
valuecheck(ddc3,ddc3_user);
valuecheck(cnstr3.relation,[Constraint.equal;Constraint.ineq]);

display('Check NonlinearConstraint with function handle');
cnstr4 = NonlinearConstraint([0;0],[0;1],-inf(3,1),ones(3,1),@cnstr_userfun4);
[c4,dc4] = cnstr4.eval([1;2;0]);
[~,dc4_val] = cnstr4.evalSparse([1;2;0]);
valuecheck(dc4,sparse(cnstr4.iCfun,cnstr4.jCvar,dc4_val,cnstr4.num_cnstr,cnstr4.xdim,cnstr4.nnz));
cnstr4 = cnstr4.setSparseStructure([1;1;2;2],[1;2;2;3],4);
[c4_val,dc4_val] = cnstr4.evalSparse([1;2;0]);
valuecheck(dc4,sparse(cnstr4.iCfun,cnstr4.jCvar,dc4_val,cnstr4.num_cnstr,cnstr4.xdim,cnstr4.nnz));
valuecheck(c4,c4_val);

display('Check LinearConstraint')
A = sparse([1;1;2;3;3;3],[1;4;2;1;2;3],randn(6,1));
cnstr5 = LinearConstraint([0;0;0],[0;1;0],-inf(4,1),zeros(4,1),A);
valuecheck(A,cnstr5.A);
x5 = randn(4,1);
[c5,dc5,ddc5] = cnstr5.eval(x5);
valuecheck(A*x5,c5);
valuecheck(dc5,A);
valuecheck(ddc5,zeros(12,4));
end

function [c,dc] = cnstr_userfun2(x)
c = [x(1)^2+2*x(1)*x(2)+x(2)^3;x(1)*x(2)^2];
dc = [2*x(1)+2*x(2) 2*x(1)+3*x(2)^2;x(2)^2 2*x(1)*x(2)];
end

function [c,dc,ddc] = cnstr_userfun3(x)
c = [x(1)^2+2*x(1)*x(2)+x(2)^3;x(1)*x(2)^2];
dc = [2*x(1)+2*x(2) 2*x(1)+3*x(2)^2;x(2)^2 2*x(1)*x(2)];
ddc = [2 2;0 2*x(2);2 6*x(2);2*x(2) 2*x(1)];
end

function [c,dc] = cnstr_userfun4(x)
c = [x(1)+x(2)^3;x(2)*x(3)];
dc = [1 3*x(2)^2 0;0 x(3) x(2)];
end