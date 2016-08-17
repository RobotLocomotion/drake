function [z,F,info,np]=testMPCC
  N = 1;
  xdim = N;
  
  
  nvars = N + xdim;% + constraint.getNumSlackVariables();
%   
  np = NonlinearProgram(nvars);
  
  %   constraint = NonlinearComplementarityConstraint(@testfun,xdim,N,3);
  constraint = LinearComplementarityConstraint(zeros(N),zeros(N,1),eye(N),1);
  
  np = np.addConstraint(constraint,(1:nvars)');
  
%   
%   
%   lincon = constraint.getLinearConstraints();
%   for k=1:length(lincon),
%     np = np.addLinearConstraint(lincon{k});
%   end
%   
%   nlncon = constraint.getNonlinearConstraints();
%   for k=1:length(nlncon),
%     np = np.addNonlinearConstraint(nlncon{k});
%   end
%   
%   bcon = constraint.getBoundingBoxConstraints();
%   for k=1:length(bcon),
%     np = np.addBoundingBoxConstraint(bcon{k});
%   end
  
  cost = FunctionHandleObjective(N + xdim,@costfun);
  np = np.addCost(cost,1:N+xdim);
  
  np = np.addConstraint(LinearConstraint(1,1,[ones(1,N) zeros(1,xdim-N) ones(1,N)]));
  
  [z,F,info] = np.solve(randn(np.num_vars,1));
%   [z,F,info] = np.solve([1;0]);
  
  function [f,df] = testfun(y)
    x = y(1:xdim);
    f = x(1:N);
    df = [eye(N) zeros(N,xdim)];
  end

  function [f,df] = costfun(y)
    x = y(1:xdim);
    z = y(xdim+1:end);
    f = 2*[ones(1,N) zeros(1,xdim-N)]*x + 3*ones(1,N)*z;
    df = [2*[ones(1,N) zeros(1,xdim-N)] 3*ones(1,N)];
  end

end