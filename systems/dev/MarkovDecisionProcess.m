classdef MarkovDecisionProcess < RobotLibSystem
% implements a model-based (transition probabilities are known)
% finite-state, finite-action, discrete time markov decision process.

% NOTEST

  properties
    S  % states; if vectors, then S(:,i) is state i
    A  % actions; if vectors, then A(:,i) is action i
    T  % T(s,a,sn) is Pr(S[n+1]=sn|S[n]=s,A[n]=a).  probably a sparse matrix
    C  % C(s,a) is Cost of being in state s and taking action a
    ts=1 % sample time (default = 1)
    gamma=1 % discount factor (default = 1)
  end
  
  methods
    function obj = MarkovDecisionProcess(S,A,T,C,gamma,ts)
      obj = obj@RobotLibSystem(0,1,size(A,1),size(S,1),false,true);
      obj.S = S;
      obj.A = A;
      obj.T = T;
      obj.C = C;
      if (nargin>4) obj.gamma = gamma; end
      if (nargin>5) obj.ts = ts; end
    end
    
    function y = output(obj,t,s,u)
      % x here is the integer discrete state
      y = obj.S(:,s);
    end

    function sn = update(obj,t,s,u)
      % lookup u in action vector
      a = find(all(abs(repmat(u,1,size(A,2))-A)>1e-8));
      if (isempty(a) || length(a)>1)
        error('Error looking up discrete action');
      end
      psn = T(s,a,:);
      sn = find(cumsum(psn)>rand(),1);
    end

    function ts = getSampleTime(obj)
      ts = [obj.ts,0];
    end
    
    function [J,PI]=valueIteration(mdp,converged)
      if (nargin<2) converged=.1; end
      nS = size(mdp.S,2); nA = size(mdp.A,2);
      J = zeros(nS,1);
      err = inf;
      
      % stack different a's on top of each other: 
      % e.g. Tstack = [T(s,a=1,sn);T(s,a=2,sn);...]
      Tstack = reshape(obj.T,nS*nA,nS);       
      
      while (err > converged)
        Jold = J;
        [J,PI] = min(C+mdp.gamma*reshape(Tstack*J,nS,nA),[],2);
        err = max(abs(Jold-J));
      end
    end
    
    function dynamicProgramming(mdp)
    end
  end
  
  methods (Static)
    function mdp = discretizeSystem(sys,xbins,ubins,ts)
    end
  end
end