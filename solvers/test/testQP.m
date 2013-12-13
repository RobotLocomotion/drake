function testQP

w = warning('off','optim:quadprog:SwitchToMedScale');

prog = QuadraticProgram(eye(2),zeros(2,1));
testSolvers(prog)

prog = QuadraticProgram(1,1,1,50,[],[],-10,10);
testSolvers(prog);

prog = QuadraticProgram(1,1,[],[],[],[],5,10);
testSolvers(prog);

warning(w);

end



function testSolvers(prog)
  [x,objval,exitflag,execution_time]=compareSolvers(prog);
  fprintf('\n\n');
  
  for i=2:length(x)
    valuecheck(x{1},x{i});
    valuecheck(objval{1},objval{i});
  end
end