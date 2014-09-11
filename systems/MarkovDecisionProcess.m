classdef MarkovDecisionProcess < DrakeSystem
% implements a model-based (transition probabilities are known)
% finite-state, finite-action, discrete time markov decision process.

% NOTEST

  properties
    S  % states; if vectors, then S(:,i) is state i
    A  % actions; if vectors, then A(:,i) is action i
    T  % T{k}(i,j) is Pr( s[n+1]=S(:,j) | s[n]=S(:,i),a[n]=A(:,k)).  can be sparse matrices
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
      obj = setSampleTime(obj,[ts;0]);
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
      psn = T{a}(s,:);
      sn = find(cumsum(psn)>rand(),1);
    end
    
    function [J,PI]=valueIteration(mdp,converged,drawfun)
      if (nargin<2) converged=.1; end
      nS = size(mdp.S,2); nA = size(mdp.A,2);
      J = zeros(nS,1);
      err = inf;
      
      % stack different a's on top of each other: 
      % e.g. Tstack = [T(s,a=1,sn);T(s,a=2,sn);...]
      Tstack = vertcat(mdp.T{:});       
      
      while (err > converged)
        Jold = J;
        [J,PI] = min(mdp.C+mdp.gamma*reshape(Tstack*J,nS,nA),[],2);
        err = max(abs(Jold-J));
        if nargin>2, drawfun(J); drawnow; end
      end
    end
    
    function dynamicProgramming(mdp)
      error('not implemented yet, but would be easy');
    end
  end
  
  methods (Static)
    function mdp = discretizeSystem(sys,costfun,xbins,ubins,options)
      % @param sys the DynamicalSystem that should be discretized
      % @param costfun is a function handle with costdot = costfun(sys,x,u)
      % @param xbins a cell array defining the (uniform) state distribution
      % @param ubins a cell array defining the (uniform) input
      % discretization
      % @option gamma @default 1;
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
      xmax = reshape(cellfun(@(a)a(end),xbins),[],1);
      
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

      % offsets for inline sub2ind
      nskip = [1,cumprod(cellfun(@length,xbins(1:end-1)))];
      function ind = mysub2ind(subinds)
        ind = 1+sum(nskip*(subinds-1)');
      end
      
      % initialize transition matrix
      [T{1:na}] = deal(sparse(ns,ns));
      
      % initialize cost
      C = zeros(ns,na);
      
      waitbar_h = waitbar(0,'Computing one-step dynamics and transition matrix...');
      waitbar_cleanup = onCleanup(@()close(waitbar_h));  % doesn't seem to catch ctrl-c, despite the documentation
      waitbar_lastupdate = 0;
      for si=1:ns
        for ai=1:na
          % todo: better than just forward euler here?
          xn = S(:,si) + options.dt*dynamics(sys,0,S(:,si),A(:,ai));
          C(si,ai) = options.dt*costfun(sys,S(:,si),A(:,ai));
          
          % wrap coordinates
          xn(options.wrap_flag) = mod(xn(options.wrap_flag)-xmin(options.wrap_flag),xmax(options.wrap_flag)-xmin(options.wrap_flag)) + xmin(options.wrap_flag);
          
          % truncate back onto the grid if it's off the edge
          xn = min(max(xn,xmin),xmax);

          % populate T using barycentric interpolation
          % simple description in Scott Davies, "Multidimensional
          % Triangulation... ", NIPS, 1996
          
          % Note: The most mature implmentation that I found quickly is in
          %   https://svn.csail.mit.edu/locomotion/robotlib/tags/version2.1/tools/@approx
          % which is called from dev/mdp.m
          % also found old non-mex version here:
          % https://svn.csail.mit.edu/locomotion/robotlib/tags/version1/approx/@barycentricGrid/state_index.m
          % but they were pretty hard to use, so I basically started over
          
          for xi=1:num_x
            % note: could do binary search here
            bin(xi) = find(xn(xi)<=xbins{xi}(2:end),1);
          end
          sidx_min = mysub2ind(bin);  % lower left
          
%          bin = bin+1; % move pointer to upper right
%          sidx_max = mysub2ind(bin);    % upper right
          sidx_max = sidx_min+sum(nskip);

          % compute relative locations
          fracway = (xn-S(:,sidx_min))./(S(:,sidx_max)-S(:,sidx_min));
          [fracway,p] = sort(fracway); % p from Davies96
          fracway(end+1)=1;
          
          % crawl through the simplex
          % start at the top-right corner
          sidx = sidx_max;  
          T{ai}(si,sidx) = fracway(1);
%          xn_recon = T{ai}(si,sidx)*S(:,sidx);  % just a sanity check
          % now move down the box as prescribed by the permutation p
          for xi=1:num_x
%            bin(p(xi))=bin(p(xi))-1;
%            sidx = mysub2ind(bin);
            sidx = sidx - nskip(p(xi));
            T{ai}(si,sidx) = fracway(xi+1)-fracway(xi);
%            xn_recon = xn_recon+T{ai}(si,sidx)*S(:,sidx);  % just a sanity check
          end
%          valuecheck(xn,xn_recon);  % just a sanity check
        end
        
        if si/ns>waitbar_lastupdate+.025 % don't call the gui too much
          waitbar(si/ns,waitbar_h);
          waitbar_lastupdate = si/ns;
        end
      end
      
      mdp = MarkovDecisionProcess(S,A,T,C,options.gamma,options.dt);
    end
  end
end
