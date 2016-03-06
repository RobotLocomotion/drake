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
      obj = obj@DrakeSystem(0,size(S,2),size(A,1),size(S,1),false,true);
      obj.S = S;
      obj.A = A;
      obj.T = T;
      obj.C = C;
      if (nargin>4) obj.gamma = gamma; end
      if (nargin<5) ts = 1; end
      obj = setSampleTime(obj,[ts;0]);
    end
    
    function s0 = getInitialState(obj)
      % discrete state
      s0 = zeros(size(obj.S,2),1);
      s0(randi(size(obj.S,2))) = 1;
    end
    
    function y = output(obj,t,s,u)
      % y is the expected state 
      y = obj.S*s;
    end

    function sn = update(obj,t,s,u)
      % lookup u in action vector
      a = find(all(abs(repmat(u,1,size(A,2))-A)>1e-8));
      if (isempty(a) || length(a)>1)
        error('Error looking up discrete action');
      end
      sn = obj.T{a}'*s;
    end
    
    function [J,PI]=valueIteration(mdp,converged,drawfun)
      if (nargin<2) converged=.1; end
      nS = size(mdp.S,2); % nS is number of discretized states
      nA = size(mdp.A,2); % nA is number of discretized actions
      J = zeros(nS,1);
      err = inf;
      
      % stack different a's on top of each other: 
      % e.g. Tstack = [T(s,a=1,sn);T(s,a=2,sn);...]
      Tstack = vertcat(mdp.T{:});       
      
      while (err > converged)
        Jold = J;
        [J,PI] = min(mdp.C+mdp.gamma*reshape(Tstack*J,nS,nA),[],2);
        err = max(abs(Jold-J));
        if nargin>2, drawfun(J,PI); drawnow; end
      end
      
      PI = MarkovDecisionProcessPolicy(mdp.A,PI);
      PI = setInputFrame(PI,getStateFrame(mdp));
      PI = setOutputFrame(PI,getInputFrame(mdp));
    end
    
    function dynamicProgramming(mdp)
      error('not implemented yet, but would be easy');
    end
  end
  
  methods (Static)
    function mdp = discretizeSystem(sys,costfun,xbins,ubins,options)
      % @param sys the DynamicalSystem that should be discretized
      % @param costfun is a function handle with costdot = costfun(sys,x,u)
      % @param xbins a cell array defining the (uniform) state
      % distribution. 
      % each cell within xbins corresponds to a dimension in
      % the state space, and the contents of the cell is an array that must
      % be sorted in increasing order with the value of the element 
      % corresponding to the discretized value of that state variable.
      % @param ubins a cell array defining the (uniform) input. 
      % similar to xbins, where each cell corresponds to a dimension in the
      % control input space and the contents of each cell must be sorted in
      % increasing order.
      % @option gamma @default 1;
      % @option dt timestep.  @default 1
      % @option wrap_flag boolean vector the same size as x which causes the
      % resulting controller to wrap around for the dimensions where
      % wrap_flag is true. @default false

      typecheck(sys,'DynamicalSystem');
      
      is_ct = isCT(sys);
      if ~is_ct && ~isDT(sys)
        error('Drake:MarkovDecisionProcess:discretizeSystem:UnsupportedSystem','only systems that are purely CT or purely DT are implemented so far');
      end      
      assert(isTI(sys),'Drake:MarkovDecisionProcess:discretizeSystem:UnsupportedSystem','only support time-invariant systems');
      
      num_x = getNumStates(sys); % num_x is the number of state variables
      num_u = getNumInputs(sys); % num_u is the number of control inputs
      if ~iscell(xbins) && num_x==1, xbins = {xbins}; end
      if ~iscell(ubins) && num_u==1, ubins = {ubins}; end
      if nargin<4, options=struct(); end
      if ~isfield(options,'dt') options.dt = 1; end
      if ~isfield(options,'wrap_flag') options.wrap_flag = false(num_x,1); end
      
      xmin = reshape(cellfun(@(a)min(a),xbins),[],1);
      xmax = reshape(cellfun(@(a)max(a),xbins),[],1);
      
      % construct the grids S and A
      assert(iscell(xbins));
      assert(iscell(ubins));
      
      Sgrid = cell(1,num_x);
      [Sgrid{:}] = ndgrid(xbins{:});
      % Sgrid{j}(i_1,i_2,...,i_{num_x}) = xbins{j}(i_j)
      S = cellfun(@(a)reshape(a,1,[]),Sgrid,'UniformOutput',false);
      S = vertcat(S{:});
      % S is a num_x by ns array, with each row corresponding to the 
      % value of the state variables for a specific possible (discretized) 
      % state of the world.

      Agrid = cell(1,num_u);
      [Agrid{:}] = ndgrid(ubins{:});
      % Similarly, Agrid{j}(i_1,i_2,...,i_{num_u}) = ubins{j}(i_j)
      A = cellfun(@(a)reshape(a,1,[]),Agrid,'UniformOutput',false);
      A = vertcat(A{:});
      % A is a num_u by na array, with each row corresponding to the value
      % of the control inputs for a specific possible (discretized) action
      % that the robot can take.
      
      ns = size(S,2); % number of discretized states
      na = size(A,2); % number of discretized actions

      % offsets for inline sub2ind
      nskip = [1,cumprod(cellfun(@length,xbins(1:end-1)))];
      function ind = mysub2ind(subinds)
        ind = 1+sum(nskip*(subinds-1)');
      end
      
      % initialize transition matrix
      % T{a_i} is the transition matrix corresponding to action a_i
      [T{1:na}] = deal(sparse(ns,ns));
      
      % initialize cost
      C = zeros(ns,na);

      % note: this can be MUCH faster if we vectorize the calls to 
      % dynamics and cost.  (that's what we used to do)
      
      waitbar_h = waitbar(0,'Computing one-step dynamics and transition matrix...');
      waitbar_cleanup = onCleanup(@()close(waitbar_h));  % doesn't seem to catch ctrl-c, despite the documentation
      waitbar_lastupdate = 0;
      for ai=1:na
        for si=1:ns
          if is_ct
            % todo: better than just forward euler here?
            xn = S(:,si) + options.dt*dynamics(sys,0,S(:,si),A(:,ai));
            C(si,ai) = options.dt*costfun(sys,S(:,si),A(:,ai));
          else % is dt
            xn = update(sys,0,S(:,si),A(:,ai));
            C(si,ai) = costfun(sys,S(:,si),A(:,ai));
          end
          
          % wrap coordinates
          xn(options.wrap_flag) = mod(xn(options.wrap_flag)-xmin(options.wrap_flag),xmax(options.wrap_flag)-xmin(options.wrap_flag)) + xmin(options.wrap_flag);
          
          [idx,coef] = barycentricInterpolation(xbins,xn);
          T{ai}(si,idx) = coef;
        end
        
        if ai/na>waitbar_lastupdate+.025 % don't call the gui too much
          waitbar(ai/na,waitbar_h);
          waitbar_lastupdate = ai/na;
        end
      end
      
      mdp = MarkovDecisionProcess(S,A,T,C,options.gamma,options.dt);
      mdp = setInputFrame(mdp,getInputFrame(sys));
      mdp = setOutputFrame(mdp,getStateFrame(sys));  % todo: should it use the outputs instead?
      
      function x_disc = stateTransform(x_cont)
        x_disc = zeros(ns,1);
        x_cont(options.wrap_flag) = mod(x_cont(options.wrap_flag)-xmin(options.wrap_flag),xmax(options.wrap_flag)-xmin(options.wrap_flag)) + xmin(options.wrap_flag);
        [idx,coef] = barycentricInterpolation(xbins,x_cont);
        x_disc(idx) = coef;
      end
      
      addTransform(getStateFrame(sys),FunctionHandleCoordinateTransform(0,0,getStateFrame(sys),getStateFrame(mdp),true,true,[],[],@(t,x,u)stateTransform(u)));
    end
  end
end
