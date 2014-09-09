classdef MarkovDecisionProcess < DrakeSystem
% implements a model-based (transition probabilities are known)
% finite-state, finite-action, discrete time markov decision process.

% NOTEST

  properties
    S  % states; if vectors, then S(:,i) is state i
    A  % actions; if vectors, then A(:,i) is action i
    T  % T(i,j,k) is Pr( s[n+1]=S(:,k) | s[n]=S(:,i),a[n]=A(:,j)).  probably a sparse matrix
    C  % C(i,j) is the cost of being in state S(:,i) and taking action A(:,j)
    gamma=1 % discount factor (default = 1)
  end
  
  methods
    function obj = MarkovDecisionProcess(S,A,T,C,gamma,ts)
      obj = obj@DrakeSystem(0,1,size(A,1),size(S,1),false,true);
      obj.S = S;
      obj.A = A;
      obj.T = T;
      obj.C = C;
      if (nargin>4) obj.gamma = gamma; end
      if (nargin<5) ts = 1; end
      obj = setSampleTime(obj,[ts,0]);
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
      error('not implemented yet, but would be easy');
    end
  end
  
  methods (Static)
    function mdp = discretizeSystem(sys,xbins,ubins,options)
      % @param sys the DynamicalSystem that should be discretized
      % @param xbins a cell array defining the (uniform) state distribution
      % @param ubins a cell array defining the (uniform) input
      % discretization
      % @option dt timestep.  @default 1
      % @option wrap_flag boolean vector the same size as x which causes the
      % resulting controller to wrap around for the dimensions where
      % wrap_flag is true. @default false

      typecheck(sys,'DynamicalSystem');
      
      assert(isCT(sys),'Drake:MarkovDecisionProcess:discretizeSystem:OnlyCTsoFar','only continuous-time systems are implemented so far');
      assert(isTI(sys),'Drake:MarkovDecisionProcess:discretizeSystem:OnlyTI','only support time-invariant systems');
      
      num_x = getNumStates(sys);
      num_u = getNumInputs(sys);
      if ~iscell(xbins) && num_x==1, xbins = {xbins}; end
      if ~iscell(ubins) && num_u==1, ubins = {ubins}; end
      if nargin<4, options=struct(); end
      if ~isfield(options,'dt') options.dt = 1; end
      if ~isfield(options,'wrap_flag') options.wrap_flag = false(num_x,1); end
      
      xmin = reshape(cellfun(@(a)a(1),xbins),[],1);
      xmax = reshape(cellful(@(a)a(end),xbins),[],1);
      
      % construct the grids
      assert(iscell(xbins));
      Sgrid = cell(1,num_x);
      [Sgrid{:}] = ndgrid(xbins{:});
      S = cellfun(@(a)reshape(a,1,[]),Sgrid,'UniformOutput',false);
      S = vertcat(S{:});

      assert(iscell(ubins));
      Agrid = cell(1,num_u);
      [Agrid{:}] = ndgrid(ubins{:});
      A = cellfun(@(a)reshape(a,1,[]),Agrid,'UniformOutput',false);
      A = vertcat(A{:});
      
      ns = size(S,2);
      na = size(A,2);
      
      % generate all possible state and action pairs
      Sall = repmat(S,1,na); % repeat s na times
      Aall = reshape(repmat(A,ns,1),1,ns*na); % repeat a ns times

      % compute the dynamics
      fprintf('Computing one-step dynamics...');
      % todo: vectorize this!
      % and possibly support something more general that defaulting to than
      % forward euler (won't work for hybrid, etc), but a call to simulate
      % would be way too expensive.
      xn = Sall;
      for i=1:size(Sall,2)
        xn(:,i) = Sall(:,i) + dt*dynamics(sys,0,Sall(:,i),Aall(:,i));
      end
      fprintf('done.\n');

      
      % Compute the transition matrix
      % Note: The most mature implmentation that I found quickly is in
      %   https://svn.csail.mit.edu/locomotion/robotlib/tags/version2.1/tools/@approx
      % which is called from dev/mdp.m 
      fprintf('Computing transition matrix...');

      % wrap coordinates
      if any(options.wrap_flag)
        for i=find(options.wrap_flag)  % almost certainly more efficient than repmat'ing xmin,xmax to call a vectorized mod
          xn(i,:) = mod(xn(i,:)-xmin(i),xmax(i)-xmin(i))+xmin(i);
        end
      end
      
      % project points off the grid back onto the edge
      for i=1:num_x
        xn(i,:) = max(xn(i,:),xmin(i));
        xn(i,:) = min(xn(i,:),xmax(i));
      end
      
      % barycentric interpolation 
      % found old non-mex version here: 
      % https://svn.csail.mit.edu/locomotion/robotlib/tags/version1/approx/@barycentricGrid/state_index.m
      % todo: vectorize (or mex) this
      for i=1:size(xn,2)
        for ci = 1:num_x
          cindy = find(xn(ci,i)<=xbins{ci}(2:end),1)-1; % find first guy it's above
          if ci==1
            sindi = cindy+1;
          else
            sindi = sindi + cindy*nskip(ci);
          end
      end
      
      fprintf('done.\n');

    end
  end
end
