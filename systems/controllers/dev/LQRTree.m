classdef LQRTree < HybridDrakeSystem

  properties
    zero_mode_num=1;
    tilqr_mode_num=2;
    num_tvlqr_modes=0;
    Vf
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
    function obj=LQRTree(tilqr,V)
      typecheck(tilqr,'AffineSystem');
      typecheck(V,'PolynomialLyapunovFunction');
      obj=obj@HybridDrakeSystem(tilqr.getNumInputs(),tilqr.getNumOutputs());
      obj = obj.setInputFrame(tilqr.getInputFrame());
      obj = obj.setOutputFrame(tilqr.getOutputFrame());
            
      % add a "do nothing" controller (aka zero controller)
      [obj,obj.zero_mode_num] = addMode(obj,ConstOrPassthroughSystem(zeros(obj.num_y,1),obj.num_u));
      
      % switch from zero control to any controller
      obj = addTransition(obj,obj.zero_mode_num,@inAnyFunnelGuard,@transitionIntoAnyFunnel,true,true);

      % add the ti controller
      [obj,obj.tilqr_mode_num] = addMode(obj,tilqr);

      % switch from ti controller to any controller
      obj = addTransition(obj,obj.tilqr_mode_num,@(obj,~,t,x)1-eval(V,t,x),@transitionIntoAnyFunnel,true,true);
      
      %% precompute quadratic forms
      % switch to object coordinates
      xVf=V.getFrame; xinf=obj.getInputFrame;
      xV=xVf.poly; xin = xinf.poly;
      if (xVf ~= xinf)
        tf=findTransform(xinf,xVf,true);  typecheck(tf,'AffineTransform');
        Vpoly = msubs(V.Vpoly,xV,tf.D*xin+tf.y0);
      else
        Vpoly = V.Vpoly;
      end
      
      if (deg(Vpoly,xV)>2) error('precomputation assumes quadratic forms for now'); end
      obj.S1=sparse(doubleSafe(.5*subs(diff(diff(Vpoly,xin)',xin),xin,0*xin)));
      obj.s2=sparse(doubleSafe(subs(diff(Vpoly,xin),xin,0*xin)));
      obj.s3=doubleSafe(subs(V.Vpoly,xin,0*xin));
      obj.x0=zeros(tilqr.getNumInputs,1); % tree controller is in the coordinate frame of the tilqr controller
      obj.t0=0;
      obj.m=obj.tilqr_mode_num;
    end
    
    function [obj,ind]=addTrajectory(obj,tvlqr,Vtv,parentID,parentT)
      typecheck(tvlqr,'LTVControl');
      typecheck(Vtv,'PolynomialTrajectory');
      
      N = obj.num_tvlqr_modes;
      if (nargin<4) parentID=obj.tilqr_mode_num; end
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
      
      % from this controller to the parent
      obj = addTransition(obj,mode_num,@(obj,t,t0,x) Vtv.tspan(end)-t+t0,@toParent,false,false);
      
      % precompute quadratic forms
      nX=length(obj.p_x);
      ts=Vtv.getBreaks();
      for i=1:length(ts)
        V=Vtv.eval(ts(i));
        x=decomp(V);
        n=length(obj.t0);
        if (deg(V,x)>2) errror('precomputation assumes quadratic forms for now'); end
        S1(:,:,i)=doubleSafe(.5*subs(diff(diff(V,x)',x),x,0*x));
        obj.S1(nX*n+(1:nX),nX*n+(1:nX))=S1(:,:,i);
        s2(1,:,i)=doubleSafe(subs(diff(V,x),x,0*x));
        obj.s2(n+1,nX*n+(1:nX))=s2(1,:,i);
        s3(i)=doubleSafe(subs(V,x,0*x));
        obj.s3(n+1,1)=s3(i);
        obj.x0(n*nX + (1:nX),1)=tvlqr.x0.eval(ts(i));
        obj.t0(n+1)=ts(i);
        obj.m(n+1)=mode_num;
      end
      
      S1pp=foh(Vtv.getBreaks(),S1);
      s2pp=foh(Vtv.getBreaks(),s2);
      s3pp=foh(Vtv.getBreaks(),s3);
      
      function V=fastVlookup(t,x,S1pp,s2pp,s3pp)
        V=x'*ppval(S1pp,t)*x+ppval(s2pp,t)*x+ppval(s3pp,t);
      end
      
      obj.Vtv = {obj.Vtv{:},@(t,x)fastVlookup(t,x,S1pp,s2pp,s3pp)};
      
      % fell out of this funnel
      obj = addTransition(obj,mode_num,@(obj,t,t0,x)1-fastVlookup(t-t0,x,S1pp,s2pp,s3pp),@transitionIntoAnyFunnel,true,false);
%      obj = addTransition(obj,mode_num,@(obj,t,t0,x)1,@transitionIntoAnyFunnel,true,false);
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
         [tmin,Vmin]=fminbnd(@(t) obj.Vtv{tvind}(t,x),a,b,options);
       end
    end
    
    function [g,dg,m,tmin]=finalTreeConstraint(obj,x)
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
        xtraj = obj.modes{m}.x0;
        
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
            
            if (min(abs(tmin-xtraj.tspan))<=options.TolX) % then I'm at one of the rails
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
      
      if (nargin<7) options = struct(); end
      if (~isfield(options,'num_samples_before_done')) options.num_samples_before_done = 100; end
      if (~isfield(options,'num_branches')) options.num_branches = inf; end
      if (~isfield(options,'verify')) options.verify = true; end        
      if (~isfield(options,'stabilize_goal')) options.stabilize_goal = true; end        
      if (~isfield(options,'plotdims')) options.plotdims = [1 2]; end
      if (~isfield(options,'xs')) options.xs=[]; end
      if (isfield(options,'con')) con=options.con; end
      
      options.method='dircol';
%      options.warning=false;
      options.plot_rho=false;
      options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,options.plotdims);  % for debugging

      nX=length(xG);
      nU=length(uG);
      
      p_nolim=setInputLimits(p,repmat(-inf,nU,1),repmat(inf,nU,1));
      
%      figure(1); clf; hold on;
      
      % compute stabilizing controller and estimate ROA
      if (options.stabilize_goal)
        disp('building and verifying time-invariant controller...')
        [ti,Vf] = tilqr(p,xG,uG,Q,R);
      else
        ti = AffineSystem([],[],[],[],[],[],[],zeros(getNumInputs(p),getNumStates(p)),uG);
        ti = setInputFrame(ti,p.getOutputFrame);
        ti = setOutputFrame(ti,p.getInputFrame);
      end
      if (isfield(options,'Vf'))
        Vf = options.Vf;
      end
      if (options.verify)
        sys = feedback(p_nolim,ti);
        psys = taylorApprox(sys,0,xG,[],3);
        Vf = regionOfAttraction(psys,xG,Vf,struct('degL1',3));
      
        if (isa(p,'PendulumPlant'))
          warning('artificially trimming pendulum tilqr ROA, since we don''t have saturations implemented yet');
          Vf = Vf*5;  % artificially prune, since ROA is solved without input limits
        end
        
        plotFunnel(Vf); drawnow;
      end
      
      % create LQR Tree
      c = LQRTree(ti,Vf);

%      options.MajorOptimalityTolerance=1e-3;
%      options.MajorFeasibilityTolerance=1e-3;
%      options.MinorFeasibilityTolerance=1e-3;
      options.MajorOptimalityTolerance=1e-2;
      options.MajorFeasibilityTolerance=1e-2;
      options.MinorFeasibilityTolerance=1e-2;
      
      num_branches=0;
      num_consecutive_samples=0;
      tf0=.5;

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
          Vmin=checkFunnels(c,xs)
          if (Vmin<1)
            % then i'm already in the basin
            num_consecutive_samples=num_consecutive_samples+1;
            if (num_consecutive_samples>=options.num_samples_before_done) break; end
            continue;
          end
          num_consecutive_samples=0;
        end
        
        % try to find a trajectory
        utraj0 = PPTrajectory(foh(linspace(0,tf0,nbreaks),randn(nU,nbreaks)));
        con.x0.lb=xs;
        con.x0.ub=xs;
        con.u.lb = p.umin;
        con.u.ub = p.umax;
%        con.xf.lb=xG;  % just grow to the goal for now
%        con.xf.ub=xG;
%        con.uf.lb=uG;
%        con.uf.ub=uG;
        con.xf.ceq=@c.finalTreeConstraint;
        [utraj,xtraj,info] = trajectoryOptimization(p,@(t,x,u)trajCost(t,x,u,.01),@trajFinalCost,xs,utraj0,con,options);
        if (info~=1) % failed to find a trajectory
          if (all(info~=[3,13,32]))
            [str,cat] = snoptInfo(info);
            warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);
          end
          num_consecutive_samples=num_consecutive_samples+1;
          if (num_consecutive_samples>=options.num_samples_before_done) break; end
          continue;
        end
        
        fnplt(xtraj,options.plotdims); drawnow;
                
        [~,~,parent_mode_num,parent_T] = finalTreeConstraint(c,xtraj.eval(xtraj.tspan(end)));
        if (parent_mode_num==c.tilqr_mode_num)
          Vparent=Vf;
        else
          Vparent=Vtv{parent_mode_num-c.tilqr_mode_num}.eval(parent_T);
          x=msspoly('x',nX+1);
          Vparent=subs(Vparent,x(2:nX+1),x(1:nX)); % yuck.  this will get undone in tvlqrClosedLoop; but I somehow need them consistent.
        end
        
        fprintf(1,'stabilizing and verifying branch %d...',num_branches+1);
        % stabilize trajectory and estimate funnel
        [tv,sys,xtraj2,utraj2,Vtraj,Vftraj] = tvlqrClosedLoop(p_nolim,xtraj,utraj,Q,R,Vparent);
        if (options.verify)
          psys = taylorApprox(sys,xtraj2,[],3);
          try 
            V=sampledFiniteTimeVerification(psys,Vftraj,Vtraj,xtraj2.getBreaks(),xtraj2,utraj2,options);
          catch ex
            if (strcmp(ex.identifier,'Drake:PolynomialTrajectorySystem:InfeasibleRho'))
              warning('Drake:LQRTree:InfeasibleRho','verification failed due to infeasible rho.  discarding trajectory.');
              continue;
            end
            rethrow(ex);  % otherwise, it's a real error
          end
          plotFunnel(V,options); drawnow;
        else
          V=PolynomialTrajectory(@(t) 1e5*Vtraj.eval(t),Vtraj.getBreaks());
        end
        fprintf(1,'done.\n');
        
        % add tv controller to the tree
        c = c.addTrajectory(tv,V,parent_mode_num,parent_T);
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
  drawnow;
  delete(h);
  J=0;
  dJ=[0*t(:);0*x(:);0*u(:)]';
end
