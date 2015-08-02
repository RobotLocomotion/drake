function testConstraint
display('Check constructing constraint with anonymous function handle without gradient');
cnstr1 = FunctionHandleConstraint(0,1,1,@(x) x+1);
cnstr1.grad_method = 'numerical';
[c1,dc1] = cnstr1.eval(1);
valuecheck(c1,2);
valuecheck(dc1,1);
valuecheck(cnstr1.lb,0);
valuecheck(cnstr1.ub,1);

display('Check constructing constraint with function handle and 1st order gradient');
cnstr2 = FunctionHandleConstraint([0;1],[0;2],2,@cnstr_userfun2);
[c2,dc2] = cnstr2.eval([1;2]);
[c2_user,dc2_user] = geval(@cnstr_userfun2,[1;2],struct('grad_method','taylorvar'));
valuecheck(c2,c2_user);
valuecheck(dc2,dc2_user);

display('Check constructing constraint with function handle and 2nd order gradient');
cnstr3 = FunctionHandleConstraint([0;1],[0;2],2,@cnstr_userfun3);
[c3,dc3,ddc3] = cnstr3.eval([1;2]);
[c3_user,dc3_user,ddc3_user] = cnstr_userfun3([1;2]);
valuecheck(c3,c3_user);
valuecheck(dc3,dc3_user);
valuecheck(ddc3,ddc3_user);

display('Check DifferentiableCnostraint with function handle');
cnstr4 = FunctionHandleConstraint([0;0],[0;1],3,@cnstr_userfun4);
valuecheck(cnstr4.ceq_idx,1);
valuecheck(cnstr4.cin_idx,2);
[c4,dc4] = cnstr4.eval([1;2;0]);
cnstr4.checkGradient(1e-4,randn(3,1));
cnstr4 = cnstr4.setSparseStructure([1;1;2;2],[1;2;2;3]);
c4_val = cnstr4.eval([1;2;0]);
valuecheck(c4,c4_val);
cnstr4.checkGradient(1e-4,randn(3,1));

display('Check LinearConstraint')
A = sparse([1;1;2;3;3;3],[1;4;2;1;2;3],randn(6,1));
cnstr5 = LinearConstraint([0;0;0],[0;1;0],A);
valuecheck(A,cnstr5.A);
x5 = randn(4,1);
[c5,dc5,ddc5] = cnstr5.eval(x5);
valuecheck(A*x5,c5);
valuecheck(dc5,A);
valuecheck(ddc5,zeros(3,16));
valuecheck(cnstr5.ceq_idx,[1;3]);
valuecheck(cnstr5.cin_idx,2);

display('Check BoundingBoxConstraint')
cnstr6 = BoundingBoxConstraint([0;1;2;3],[1;2;2;3]);
valuecheck(cnstr6.lb,[0;1;2;3]);
valuecheck(cnstr6.ub,[1;2;2;3]);


display('Check setBounds')
cnstr4 = cnstr4.setBounds([1;2],[3;2]);
valuecheck(cnstr4.lb,[1;2]);
valuecheck(cnstr4.ub,[3;2]);
valuecheck(cnstr4.cin_idx,1);
valuecheck(cnstr4.ceq_idx,2);
err = struct('identifier',[]);
try
  cnstr4 = cnstr4.setBounds([2;2],[1;2]);
catch err
end
if(isempty(err.identifier))
  error('The setBounds method cannot catch the error when lb is larger than ub');
end
err = struct('identifier',[]);
try
  cnstr4 = cnstr4.setBounds([1;2;3],[3;4;5]);
catch err
end
if(isempty(err.identifier))
  error('The setBounds method cannot catch the error when the constraint changes the row size');
end
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
