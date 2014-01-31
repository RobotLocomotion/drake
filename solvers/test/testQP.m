function testQP

w = warning('off','optim:quadprog:SwitchToMedScale');

fprintf('****************************************\n');
fprintf(' min_x (.5*x''*A*x + x)\n');
fprintf('****************************************\n');
a = randn(2); 
prog = QuadraticProgram(a'*a+eye(2),randn(2,1));
testSolvers(prog)

fprintf('****************************************\n');
fprintf(' min_x (.5*x^2 + x), subj to -10<=x<=10\n');
fprintf('****************************************\n');
prog = QuadraticProgram(1,1,1,50,[],[],-10,10);
testSolvers(prog);

fprintf('****************************************\n');
fprintf(' min_x (.5*x^2 + x), subj to 5<=x<=10\n');
fprintf('****************************************\n');
prog = QuadraticProgram(1,1,[],[],[],[],5,10);
testSolvers(prog);

warning(w);

end



function testSolvers(prog)
  [x,objval,exitflag,execution_time]=compareSolvers(prog);
  fprintf('\n\n');
  
  for i=2:length(x)
    if isempty(x{i}), continue; end
    valuecheck(x{1},x{i},1e-4);
    valuecheck(objval{1},objval{i},1e-4);
  end
end