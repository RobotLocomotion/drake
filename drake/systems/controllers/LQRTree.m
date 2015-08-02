classdef LQRTree < HybridDrakeSystem

  properties
    zero_mode_num=1;
    tilqr_mode_num=2;
    num_tvlqr_modes=0;

    xG
    uG
    Vf
    xtraj={};
    utraj={};
    Vtv={};
    
    % precompute some values to optimize speed of guards and transitions
    S1=[];
    s2=[];
    s3=[];
    x0=[];
    t0=[];
    m=[];
  end
  
  methods
    function obj=LQRTree(xG,uG,tilqr,V)
      typecheck(tilqr,'AffineSystem');
      typecheck(V,'PolynomialLyapunovFunction');
      obj=obj@HybridDrakeSystem(tilqr.getNumInputs(),tilqr.getNumOutputs());
      obj = obj.setInputFrame(tilqr.getInputFrame());
      obj = obj.setOutputFrame(tilqr.getOutputFrame());
            
      typecheck(xG,'Point');
      typecheck(uG,'Point');
      obj.xG = xG;
      obj.uG = uG;
      
      % add a "do nothing" controller (aka zero controller)
      c0=ConstOrPassthroughSystem(zeros(obj.num_y,1),obj.num_u);
      c0=c0.setInputFrame(tilqr.getInputFrame());
      c0=c0.setOutputFrame(tilqr.getOutputFrame());
      [obj,obj.zero_mode_num] = addMode(obj,c0);
      
      % switch from zero control to any controller
      obj = addTransition(obj,obj.zero_mode_num,@inAnyFunnelGuard,@transitionIntoAnyFunnel,true,true);

      % add the ti controller
      [obj,obj.tilqr_mode_num] = addMode(obj,tilqr);

      % switch to object coordinates
      V = V.inFrame(obj.getInputFrame);
      
      % switch from ti controller to any controller
      obj = addTransition(obj,obj.tilqr_mode_num,@(obj,t,~,x)1-eval(V,t,x),@transitionIntoAnyFunnel,true,true);
      
      %% precompute quadratic forms
      if ~isTI(V) error('i''ve assumed so far that the initial controller / lyapunov pair is time invariant'); end
      V = V.extractQuadraticLyapunovFunction();
      obj.S1=V.S;
      obj.s2=V.s1';
      obj.s3=V.s2;
      obj.t0=0;
      obj.x0=double(obj.xG.inFrame(obj.getInputFrame));
      obj.m=obj.tilqr_mode_num;
    end
    
    function [obj,ind]=addTrajectory(obj,xtraj,utraj,tvlqr,Vtv,parentID,parentT)
      typecheck(xtraj,'Trajectory');
      typecheck(utraj,'Trajectory');
      xtraj = xtraj.inFrame(obj.getInputFrame);
      utraj = utraj.inFrame(obj.getOutputFrame);
      obj.xtraj = vertcat(obj.xtraj(:),{xtraj});
      obj.utraj = vertcat(obj.utraj(:),{utraj});
      
      typecheck(tvlqr,'AffineSystem');
      typecheck(Vtv,'QuadraticLyapunovFunction');
      
      N = obj.num_tvlqr_modes;
      if (nargin<6) parentID=obj.tilqr_mode_num; end
      typecheck(parentID,'double');
      if (parentID<obj.tilqr_mode_num || parentID>(obj.tilqr_mode_num+obj.num_tvlqr_modes)) error('invalid parent ID'); end
      
      % todo: check here that trajectories for the controller and lyapunov
      % function match?
%      Vtv_faster = flipToPP(Vtv);
      
      
      [obj,mode_num] = addMode(obj,tvlqr);
      obj.num_tvlqr_modes = obj.num_tvlqr_modes+1;
      
      function [xn,to_mode,status]=toParent(~,t,~,~,~)
        to_mode=parentID;
        fprintf(1,'t=%f, switching to parent mode %d\n',t,to_mode);
        status=0;
        if (parentID==2) % ti
          xn=[];
        elseif (parentID>2) % tv
          xn=parentT;
        end
      end
      
      if isTI(Vtv) error('assuming tv quadratic form'); end
      % from this controller to the parent
      tspan = Vtv.S.tspan;
      obj = addTransition(obj,mode_num,@(obj,t,~,x) tspan(end)-t,@toParent,false,false);
%      obj = addTransition(obj,mode_num,@(obj,t,t0,x) tspan(end)-t+t0,@toParent,false,false);
      
      % precompute quadratic forms
      Vtv = Vtv.inFrame(obj.getInputFrame);
      nX=obj.getNumInputs();
      ts=Vtv.S.getBreaks();
      for i=1:length(ts)
        n=length(obj.t0);
        obj.S1(nX*n+(1:nX),nX*n+(1:nX))=Vtv.S.eval(ts(i));
        obj.s2(n+1,nX*n+(1:nX))=Vtv.s1.eval(ts(i))';
        obj.s3(n+1,1)=Vtv.s2.eval(ts(i));
        obj.t0(n+1)=ts(i);
        obj.x0(:,n+1)=xtraj.eval(ts(i));
        obj.m(n+1)=mode_num;
      end
      
      obj.Vtv = vertcat(obj.Vtv(:),{Vtv});
      
      % fell out of this funnel
      obj = addTransition(obj,mode_num,@(obj,t,~,x)1-Vtv.eval(t,x),@transitionIntoAnyFunnel,true,false);
%      obj = addTransition(obj,mode_num,@(obj,t,t0,x)1-Vtv.eval(t-t0,x),@transitionIntoAnyFunnel,true,false);
    end
    
    function ts=getSampleTime(obj)
      ts=[-1;0];
    end
  
    function phi=inAnyFunnelGuard(obj,t,~,x)  % note: this is expensive for a guard
      phi = checkFunnels(obj,x)-1;
    end
    
    function [t0,to_mode_num,status]=transitionIntoAnyFunnel(obj,m,t,~,x)
      status=0;
      [Vmin,to_mode_num,tmin]=checkFunnels(obj,x);
      if (Vmin>1) % then I'm not in any funnel
        t0=[];
        to_mode_num=1;  % put me in the zero controller
      elseif to_mode_num==2
        t0=[];  % don't need (can't have) the starting time for the ti controller
      else % landing in a tv funnel
        t0=t-tmin;
      end
      fprintf(1,'t=%f, switching to mode %d\n',t,to_mode_num);
    end
    
    function [Vmin,mode,tmin] = checkFunnels(obj,x)
      % find lowest V for this x, by searching over ti and tv funnels.
      
      % use precomputed quadratic forms for fast lookup
      nX = length(x);
      N = length(obj.t0);
      xcopy=reshape(x(:,ones(1,N)),nX*N,1);
      V=sum(reshape((obj.S1*xcopy).*xcopy,nX,N),1)'+obj.s2*xcopy+obj.s3;
      [Vmin,imin]=min(V);
      mode=obj.m(imin);
      tmin=obj.t0(imin);
      
%       % check ti funnel first
%       Vmin=doubleSafe(subs(obj.Vf,obj.p_x,x-obj.modes{2}.x0));
%       mode=2;
%       tmin=0;
%       
%       % now check the tv funnels (note: could make this faster by precomputing)
%       for i=1:obj.num_tvlqr_modes
%         ts = obj.Vtv{i}.getBreaks();
%         for j=1:length(ts)
%           V(j)=obj.Vtv{i}.polyeval(ts(j),x-obj.modes{obj.tilqr_mode_num+i}.x0.eval(ts(j)));
%         end
%         [Vm,ind] = min(V);
%         if (Vm<Vmin)
%           Vmin=Vm;
%           mode=i+2;
%           tmin=ts(ind);
%         end
%       end


       % perform a line search here to improve tmin
       if(imin>1 && obj.m(imin-1)==mode) a=obj.t0(imin-1); else a=obj.t0(imin); end
       if(imin<N && obj.m(imin+1)==mode) b=obj.t0(imin+1); else b=obj.t0(imin); end
        
       if (a~=b)
         options = optimset('fminbnd');
         options.TolX = 1e-10;
         tvind = mode-obj.tilqr_mode_num;
         [tmin,Vmin]=fminbnd(@(t) eval(obj.Vtv{tvind},t,x),a,b,options);
       end
    end
    
    function [g,dg,m,tmin]=onTreeConstraint(obj,x)
      
      % note: the input x is in the plant state frame (everything else inside this
      % class is stored in the input frame of the controller)
      
      x = x-double(obj.xG);  % convert from plant state frame to input frame
      % note that I enforced earlier that tf was of the form y=x-c
      % (otherwise it would change the gradients below)
      
      nX = length(x);
      N = length(obj.t0);
      xbar=x(:,ones(1,N))-reshape(obj.x0,nX,N);
      d=sum(xbar.^2,1);
      [dmin,imin]=min(d);
      
      m=obj.m(imin);
      tmin=0;
      if (m==obj.tilqr_mode_num) % closest point is goal
        g = xbar(:,imin);
        dg = eye(nX);
      else % closest point is on a trajectory
        % look just ahead and behind this node to estimate best segment
        xtraj = obj.xtraj{m-obj.tilqr_mode_num};
        
        if(imin>1 && obj.m(imin-1)==m) a=obj.t0(imin-1); else a=obj.t0(imin); end
        if(imin<N && obj.m(imin+1)==m) b=obj.t0(imin+1); else b=obj.t0(imin); end
        
        if (a==b)
          g=xbar(:,imin);
          dg = eye(nX);
        else
          options = optimset('fminbnd');
          options.TolX = 1e-10;
          [tmin,dmin]=fminbnd(@(t) sum((x-xtraj.eval(t)).^2,1),a,b,options);
          xmin = xtraj.eval(tmin);
          g=x-xmin;
          
          if (nargout>1)
            xdottraj = fnder(xtraj);
            xdotmin = xdottraj.eval(tmin);
            
            tspan=xtraj.tspan;
            if (min(abs(tmin-tspan))<=options.TolX) % then I'm at one of the rails
              dtmindx = zeros(1,nX);
            else
              xddotmin = xdottraj.deriv(tmin);
              dtmindx = -xdotmin'/(g'*xddotmin - xdotmin'*xdotmin);
            end
            
            dg = eye(nX)-xdotmin*dtmindx;
          end
        end
      end        
      
      % todo: need to check for any children trajectories, entering this node, too.
    end
    
  end % end methods

  methods (Static=true)
    function c=buildLQRTree(p,xG,uG,xSampleDistFun,Q,R,options)
      %   Runs the LQR tree algorithm using polynomial tools for verification.
      %
      %   Usage:
      %     controller = buildLQRTree(sys,xG,uG,xSampleDistFun,Q,R,options)
      %
      %   Inputs:
      %     sys - DynamicalSystem object
      %     xG,uG - describes the goal.  sys.dynamics(0,xG,uG) must equal 0.
      %     xSampleDistFun - function handle to draw a random sample
      %     Q,R - default values used for both LTI and LTV stabilization
      %
      %   Outputs:
      %     controller - a hybrid LQR Tree controller object
      %
      %   Options:
      %     num_samples_before_done - number of consecutive successes for estimated
      %        probabilistic completeness (default 1000).
      %     num_branches          - number of trajectories to add (default inf)
      %     t_max               - longest trajectory (default 10s)
      %     plot_trajectories  - default true
      %     plot_basins        - default false
      %     plot_dims          - default [1,2].  (but can be multiple rows for
      %                                           multiple subplots)
      %     plant_order        - default is 3
      %     degL1        - default is sampledFiniteTimeInvariance default
      %
      %     con    - trajectory optimization constraints
      %
      %     xs,Tslb,Tsub - use these as initial samples first
      %     stabilize_goal - default is true 
      %     Vf - default final condition
      %     verify - default is true (occasionally useful for faster debugging)
      
      typecheck(p,'DynamicalSystem');
      nX=p.getNumStates();
      nU=p.getNumInputs();
      typecheck(xG,'Point');
      typecheck(uG,'Point');
      typecheck(xSampleDistFun,'function_handle');
      typecheck(Q,'numeric');
      sizecheck(Q,[nX,nX]);
      typecheck(R,'numeric');
      sizecheck(R,[nU,nU]);
      
      if (nargin<7) options = struct(); end
      if (~isfield(options,'num_samples_before_done')) options.num_samples_before_done = 100; end
      if (~isfield(options,'num_branches')) options.num_branches = inf; end
      if (~isfield(options,'verify')) options.verify = true; end        
      if (~isfield(options,'stabilize')) options.stabilize = true; end        
      if (~isfield(options,'plotdims')) options.plotdims = [1 2]; end
      if (~isfield(options,'xs')) options.xs=[]; end
      if (isfield(options,'con')) con=options.con; end
      
      options.method='dircol';
%      options.warning=false;
      options.plot_rho=false;
      options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,options.plotdims);  % for debugging

      if ~isTI(p) error('time-varying plants not supported (yet - wouldn''t be too hard)'); end
      p_nolim=setInputLimits(p,repmat(-inf,nU,1),repmat(inf,nU,1));
      
      figure(1); clf; hold on;
      
      % compute stabilizing controller and estimate ROA
      if (options.stabilize)
        disp('building and verifying time-invariant controller...')
        if (options.verify)
          [ti,Vf] = tilqr(p,xG,uG,Q,R);
        else
          ti = tilqr(p,xG,uG,Q,R);
          Vf = QuadraticLyapunovFunction(p.getStateFrame,Q);
        end
      else
        ti = ConstOrPassthroughSystem(uG,nX);
        ti = setInputFrame(ti,p.getOutputFrame);
        ti = setOutputFrame(ti,p.getInputFrame);
        Vf = QuadraticLyapunovFunction(p.getStateFrame,Q*inf);
      end
      if (isfield(options,'Vf'))
        Vf = options.Vf;
      end
      if (options.verify)
        sys = feedback(p_nolim,ti);
        psys = taylorApprox(sys,0,xG,[],3);
        Vf = regionOfAttraction(psys,Vf,struct('degL1',3));
      
        if (isa(p,'PendulumPlant'))
          warning('artificially trimming pendulum tilqr ROA, since we don''t have saturations implemented yet');
          Vf = Vf*5;  % artificially prune, since ROA is solved without input limits
        end
        
        plotFunnel(Vf.inFrame(p.getStateFrame)); drawnow;
      end
      
      % create LQR Tree
      c = LQRTree(xG,uG,ti,Vf);

      if ~isfield(options,'MajorOptimalityTolerance') options.MajorOptimalityTolerance=1e-2; end
      if ~isfield(options,'MajorFeasibilityTolerance') options.MajorFeasibilityTolerance=1e-2; end
      if ~isfield(options,'MinorFeasibilityTolerance') options.MinorFeasibilityTolerance=1e-2; end
      
      num_branches=0;
      num_consecutive_samples=0;
      tf0=.5;

      if (xG.getFrame()~=p.getStateFrame()) error('oops.  I assumed this below'); end
      
      while (true)
        if (isempty(options.xs))
          xs=xSampleDistFun();
          nbreaks=11;
          con.T.lb = .1;
          %        con.T.ub = nbreaks/10;  % effective dt=.1
          con.T.ub = .4;
          options.MajorIterationsLimit=50;
          options.MinorIterationsLimit=100;
        else
          xs=options.xs(:,1);
          options.xs = options.xs(:,2:end);
          nbreaks=21;
          con.T.lb=options.Tslb;
          con.T.ub=options.Tsub;
          options.MajorIterationsLimit=1000;
          options.MinorIterationsLimit=500;
        end
        
        if (options.verify)
          Vmin=checkFunnels(c,xs-double(xG))
          if (Vmin<1)
            % then i'm already in the basin
            num_consecutive_samples=num_consecutive_samples+1;
            if (num_consecutive_samples>=options.num_samples_before_done) break; end
            continue;
          end
          num_consecutive_samples=0;
        end
        
        % try to find a trajectory
        prog = DircolTrajectoryOptimization(p,nbreaks,[options.Tslb,options.Tsub]);
        prog = prog.addStateConstraint(ConstantConstraint(xs),1);
        prog = prog.addStateConstraint(FunctionHandleConstraint(0,0,nX,@c.onTreeConstraint),nbreaks);
        prog = prog.addRunningCost(@(t,x,u)trajCost(t,x,u,.01));
        prog = prog.addFinalCost(@trajFinalCost);
        [xtraj,utraj,~,~,info] = prog.solveTraj(tf0);
        if (info~=1) % failed to find a trajectory
          if (all(info~=[3,13,32]))
            [str,cat] = snoptInfo(info);
            warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);
          end
          num_consecutive_samples=num_consecutive_samples+1;
          if (num_consecutive_samples>=options.num_samples_before_done) break; end
          continue;
        end
        
        sfigure(1);
        fnplt(xtraj,options.plotdims); drawnow;
                
        tspan = xtraj.tspan;
        [~,~,parent_mode_num,parent_T] = onTreeConstraint(c,xtraj.eval(tspan(end)));
        xtraj = xtraj.shiftTime(parent_T-tspan(end));
        utraj = utraj.shiftTime(parent_T-tspan(end));
        if (parent_mode_num==c.tilqr_mode_num)
          Vparent=Vf;
        else
          Vparent=SpotPolynomialLyapunovFunction(Vf.getFrame(),Vtv{parent_mode_num-c.tilqr_mode_num}.inFrame(Vf.getFrame).getPoly(parent_T));
        end
        
        fprintf(1,'stabilizing and verifying branch %d...',num_branches+1);

        if (options.stabilize)
          % stabilize trajectory and estimate funnel
          [tv,Vtraj] = tvlqr(p_nolim,xtraj,utraj,Q,R,Vparent);
        else
          tv = ConstOrPassthroughSystem(uG,nX);
          tv = setInputFrame(tv,p.getOutputFrame);
          tv = setOutputFrame(tv,p.getInputFrame);
          Vtraj = QuadraticLyapunovFunction(p.getStateFrame,Q*inf);
        end
        if (options.verify)
          psys = taylorApprox(feedback(p_nolim,tv),xtraj,[],3);
          try 
            V=sampledFiniteTimeVerification(psys,xtraj.getBreaks(),Vparent,Vtraj,options);
          catch ex
            if (strcmp(ex.identifier,'Drake:PolynomialTrajectorySystem:InfeasibleRho'))
              warning('Drake:LQRTree:InfeasibleRho','verification failed due to infeasible rho.  discarding trajectory.');
              continue;
            end
            rethrow(ex);  % otherwise, it's a real error
          end
          sfigure(1);
          plotFunnel(V.inFrame(p.getStateFrame),options); drawnow;
        else
          V=1e5*Vtraj;
        end
        fprintf(1,'done.\n');
        
        % add tv controller to the tree
        c = c.addTrajectory(xtraj,utraj,tv,V,parent_mode_num,parent_T);
        Vtv{c.num_tvlqr_modes}=V;
        
        num_branches=num_branches+1;
        if (num_branches>=options.num_branches) break; end
        if (num_branches==3)
          options=rmfield(options,'trajectory_cost_fun'); % stop plotting everything
        end
      end
    end % end buildLQRTree
    
  end % end static methods
  

  
end


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

function [g,dg] = trajCost(t,x,u,R)
  g = u'*R*u;
  dg = [0,0*x',2*u'*R];
end
function [h,dh] = trajFinalCost(t,x)
  h = t;
  dh = [1,0*x'];
end

function [J,dJ]=plotDircolTraj(t,x,u,plotdims)
  h=plot(x(plotdims(1),:),x(plotdims(2),:),'.-');
  axis(2*[-pi pi -pi pi]);  % pendulum specific
  drawnow;
  delete(h);
  J=0;
  dJ=[0*t(:);0*x(:);0*u(:)]';
end
