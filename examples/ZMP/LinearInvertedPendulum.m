classdef LinearInvertedPendulum < LinearSystem
  % This is the three-dimensional model of the ZMP dynamics of the linear
  % inverted pendulum as described in Kajita03.
  %
  % - Here the input is the commanded linear *accelerations* of 
  % the center of mass (note that Kajita03 uses the jerk, but I don't think
  % it's warranted).  
  % - The state is [x,y,xdot,ydot]^T.  
  % - The output is the state and the [x,y] position of the ZMP.  
  
  methods 
    function obj = LinearInvertedPendulum(h,g)
      % @param h the (constant) height of the center of mass
      % @param g the scalar gravity @default 9.81
      if (nargin<2) g=9.81; end
      if(isa(h,'numeric'))
        rangecheck(h,0,inf);
        Czmp = [eye(2),zeros(2)];
        C = [eye(4);Czmp];
        Dzmp = -h/g*eye(2);
        D = [zeros(4,2);Dzmp];
      elseif(isa(h,'PPTrajectory'))
        Czmp = [eye(2),zeros(2)];
        C = [eye(4);Czmp]
        h_breaks = h.getBreaks;
        ddh = fnder(h,2);
        Dzmp_val = -h.eval(h_breaks)./(g+ddh.eval(h_breaks));
        D_val = zeros(6,2,length(h_breaks));
        D_val(5,1,:) = Dzmp_val; D_val(6,2,:) = Dzmp_val;
        D = PPTrajectory(spline(h_breaks,D_val));
      else
        error('LinearInvertedPendulum: unknown type for argument h.')
      end
             
      obj = obj@LinearSystem([zeros(2),eye(2);zeros(2,4)],[zeros(2);eye(2)],[],[],C,D);
      obj.h = h;
      obj.g = g;
      
      cartstateframe = CartTableState;
      obj = setInputFrame(obj,CartTableInput);
      sframe = SingletonCoordinateFrame('LIMPState',4,'x',{'x_com','y_com','xdot_com','ydot_com'});
      if isempty(findTransform(sframe,cartstateframe))
        addTransform(sframe,AffineTransform(sframe,cartstateframe,sparse([7 8 15 16],[1 2 3 4],[1 1 1 1],16,4),zeros(16,1)));
        addTransform(cartstateframe,AffineTransform(cartstateframe,sframe,sparse([1 2 3 4],[7 8 15 16],[1 1 1 1],4,16),zeros(4,1)));
      end
      obj = setStateFrame(obj,sframe);
      
      zmpframe = CoordinateFrame('ZMP',2,'z',{'x_zmp','y_zmp'});
      obj = setOutputFrame(obj,MultiCoordinateFrame({sframe,zmpframe}));
    end
    
    function v = constructVisualizer(obj)
      v = CartTableVisualizer;
    end
    
    function varargout = lqr(obj,com0)
      % objective min_u \int dt [ x_zmp(t)^2 ] 
      varargout = cell(1,nargout);
      options.Qy = diag([0 0 0 0 1 1]);
      if (nargin<2) com0 = [0;0]; end
      [varargout{:}] = tilqr(obj,Point(obj.getStateFrame,[com0;0*com0]),Point(obj.getInputFrame),zeros(4),zeros(2),options);
    end
    
    function [c,Vt] = ZMPtracker(obj,dZMP,options)
      if nargin<3 options = struct(); end
      if ~isfield(options,'use_tvlqr') options.use_tvlqr = false; end
      if ~options.use_tvlqr
        if isfield(options,'dCOM') || ~isTI(obj) || ~isa(dZMP,'PPTrajectory')
          warning('closed-form solution not implemented for these options (yet)');
          options.use_tvlqr = true;
        end
      end        
      
      typecheck(dZMP,'Trajectory');
      dZMP = dZMP.inFrame(desiredZMP);
      
      zmp_tf = dZMP.eval(dZMP.tspan(end));
      if(isTI(obj))
        [~,V] = lqr(obj,zmp_tf);
      else
        % note: i've merged this in from hongkai (it's a time crunch!), but i don't agree with
        % it. it's bad form to assume that the tv system is somehow stationary after D.tspan(end).  
        % - Russ
        valuecheck(dZMP.tspan(end),obj.D.tspan(end)); % assume this
        t_end = obj.D.tspan(end);
        ti_obj = LinearSystem(obj.Ac.eval(t_end),obj.Bc.eval(t_end),[],[],obj.C.eval(t_end),obj.D.eval(t_end));
        ti_obj = setStateFrame(ti_obj,obj.getStateFrame());
        ti_obj = setInputFrame(ti_obj,obj.getInputFrame());
        ti_obj = setOutputFrame(ti_obj,obj.getOutputFrame());
        Q = zeros(4);
  		  options.Qy = diag([0 0 0 0 1 1]);
        [~,V] = tilqr(ti_obj,Point(ti_obj.getStateFrame,[zmp_tf;0*zmp_tf]),Point(ti_obj.getInputFrame),Q,zeros(2),options);
      end
      
      options.tspan = linspace(dZMP.tspan(1),dZMP.tspan(2),10);
      options.sqrtmethod = false;
      if(isfield(options,'dCOM'))
%        options.dCOM = setOutputFrame(options.dCOM,obj.getStateFrame);
        options.dCOM = options.dCOM.inFrame(obj.getStateFrame);
        options.xdtraj = options.dCOM;
        Q = eye(4);
      else
        Q = zeros(4);
      end

      if options.use_tvlqr
        dZMP = setOutputFrame(dZMP,getFrameByNum(getOutputFrame(obj),2)); % override it before sending in to tvlqr
        % note: the system is actually linear, so x0traj and u0traj should
        % have no effect on the numerical results (they just set up the
        % frames)
        x0traj = setOutputFrame(ConstantTrajectory([zmp_tf;0;0]),obj.getStateFrame);
        u0traj = setOutputFrame(ConstantTrajectory([0;0]),obj.getInputFrame);
        options.Qy = diag([0 0 0 0 1 1]);
        options.ydtraj = [x0traj;dZMP];
        WS = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
        [c,Vt] = tvlqr(obj,x0traj,u0traj,Q,zeros(2),V,options);
        warning(WS);        
      else
        % closed-form solution, see derivation in zmp_riccati.pdf

        [breaks,coefs,n,k,d] = unmkpp(dZMP.pp);
        assert(prod(d)==2);
        c = reshape(coefs,[2,n,k]);
        c(:,:,1) = c(:,:,1) - repmat(zmp_tf,1,n);  % switch to zbar coordinates
        dt = diff(breaks);

        hg = obj.h/obj.g;
        A = obj.Ac; B = obj.Bc; S = V.S;
        Q = diag([1 1 0 0]);
        R = hg^2*eye(2); Ri = inv(R);
        N = -hg*[eye(2);zeros(2)];

        A2 = (N+S*B)*Ri*B' - A';
        B2 = [-2*eye(2); zeros(2)] + 2*hg*(N + S*B)*Ri*eye(2);
        
        alpha = zeros(4,n);
        beta = zeros(4,n,k);
        for j=n:-1:1
          beta(:,j,1) = B2*c(:,j,1);
          for i=2:k
            beta(:,j,i) = ( A2*beta(:,j,i-1) + B2*c(:,j,i-1) )/(i+1);
          end
          if (j==n) 
            s1dt = zeros(4,1);
          else
            s1dt = alpha(:,j+1) + beta(:,j+1,1);
          end
          alpha(:,j) = expm(A2*dt(j)) \ (s1dt - squeeze(beta(:,j,:))*(dt(j).^(0:k-1)'));
        end
        
        c = lqr(obj,zmp_tf);  % placeholder until I do the right thing here
        if (nargout>1)
          % note: writing directly on top of V's elements might not be
          % allowed in the future (it probably shouldn't be!)
          Vt = V;
          Vt.s1 = FunctionHandleTrajectory(@(t)s1ClosedFormRiccati(t,breaks,A2,alpha,beta), 4, breaks);
          %  Vt.s2 is left alone (until I finish this)
        end
      end
      
      function s1 = s1ClosedFormRiccati(t,breaks,A2,alpha,beta)
        j = find(t>=breaks(1:end-1),1,'last');
        if isempty(j), j=length(breaks)-1; end
        trel = t-breaks(j);
        s1 = expm(A2*trel)*alpha(:,j) + squeeze(beta(:,j,:))*(trel.^(0:size(beta,3)-1)');
      end
    end
    
    function comtraj = COMplanFromTracker(obj,com0,comdot0,tspan,c)
      doubleIntegrator = LinearSystem([zeros(2),eye(2);zeros(2,4)],[zeros(2);eye(2)],[],[],eye(4),zeros(4,2));
      doubleIntegrator = setInputFrame(doubleIntegrator,getInputFrame(obj));
      doubleIntegrator = setStateFrame(doubleIntegrator,getStateFrame(obj));
      doubleIntegrator = setOutputFrame(doubleIntegrator,getStateFrame(obj));  % looks like a typo, but this is intentional: output is only the LIMP state
      sys = feedback(doubleIntegrator,c);
      
      comtraj = simulate(sys,tspan,[com0;comdot0]);
      comtraj = inOutputFrame(comtraj,sys.getOutputFrame);
      comtraj = comtraj(1:2);  % only return position (not velocity)
    end
    
    function comtraj = COMplan(obj,com0,comf,dZMP,options)
      % implements analytic method from Harada06
      %  notice the different arugments (comf instead of comdot0)
      sizecheck(com0,2);
      sizecheck(comf,2);
      typecheck(dZMP,'PPTrajectory');  
      dZMP = dZMP.inFrame(desiredZMP);
      if ~isnumeric(obj.h) error('variable height not implemented yet'); end
      
      valuecheck(obj.g,9.81);
 
      comtraj = LinearInvertedPendulum.COMtrajFromZMP(obj.h,com0,comf,dZMP.pp);
%      com_pp = LinearInvertedPendulum.COMsplineFromZMP(obj.h,com0,comf,dZMP.pp);
%      comtraj = PPTrajectory(com_pp);
    end
    
    function comtraj = ZMPplanner(obj,com0,comdot0,dZMP,options)
      warning('Drake:DeprecatedMethod','The ZMPPlanner function is deprecated.  Please use COMplan (using the closed-form solution) or ZMPPlanFromTracker'); 
    
      % got a com plan from the ZMP tracking controller
      if(nargin<5) options = struct(); end
      c = COMtracker(obj,dZMP,options);
      comtraj = COMplanFromTracker(obj,com0,comdot0,dZMP.tspan,c);
    end
    
    
  end
  
  methods (Static)
    function comtraj = COMtrajFromZMP(h,com0,comf,zmp_pp)
      [breaks,coefs,l,k,d] = unmkpp(zmp_pp);
      for i=1:2
        this_zmp_pp = mkpp(breaks,coefs(i:2:end,:));
        ct{i} = LinearInvertedPendulum2D.COMtrajFromZMP(h,com0(i),comf(i),this_zmp_pp);
      end
      comtraj = vertcat(ct{1},ct{2});
    end
    
    function com_pp = COMsplineFromZMP(h,com0,comf,zmp_pp)
      % fast method for computing closed-form ZMP solution (from Harada06)
      % note: there is no error checking on this method (intended to be fast).
      % use ZMPplan to be more safe.
      
      [breaks,coefs,l,k,d] = unmkpp(zmp_pp);
      
      for i=1:2
        this_zmp_pp = mkpp(breaks,coefs(i:2:end,:));
        [V,W,A,Tc,~,comdot0,comdotf] = LinearInvertedPendulum2D.AnalyticZMP_internal(h,com0(i),comf(i),this_zmp_pp);
      
        % equation (5)
        % Note: i could return a function handle trajectory which is exact
        % (with the cosh,sinh terms), but for now I think it's sufficient and more
        % practical to make a new spline.
        com_knots(i,:) = [comdot0,V+A(1,:),comf(i),comdotf];
      end
      com_pp = csape(breaks,com_knots,[1 1]);
      
      % NOTEST
    end
    
    function horz_comddot_d = COMaccelerationFromZMP(h,com0,comf,dZMP)
      % fast method for computing instantaneous COM acceleration (using
      % Harada06)

      [breaks,coefs,l,k,d] = unmkpp(zmp_pp);
      
      for i=1:2
        this_zmp_pp = mkpp(breaks,coefs(i,:,:),1);
        [V,W,A,Tc] = LinearInvertedPendulum2D.AnalyticZMP_internal(h,com0(i),comf(i),this_zmp_pp);
        
        horz_comddot_d(i) = V(1)*Tc^2 + 2*A(3,1);
      end
      
      % NOTEST
    end
    
    function runPassive
      r = LinearInvertedPendulum(1.055);
      v = r.constructVisualizer();
      ytraj = r.simulate([0 3],[0;0;.1;0]);
      v.playback(ytraj);
    end
    
    function runLQR
      r = LinearInvertedPendulum(1.055);
      c = lqr(r);
      output_select(1).system = 1;
      output_select(1).output = 1;
      sys = mimoFeedback(r,c,[],[],[],output_select);
      
      v = r.constructVisualizer();

      for i=1:5;
        ytraj = sys.simulate([0 5],randn(4,1));
        v.playback(ytraj);
      end
    end
    
    function ZMPtrackingDemo()
      r = LinearInvertedPendulum(1.055);
      ts = linspace(0,8,100); 
      dZMP = setOutputFrame(PPTrajectory(spline(ts,[.08*sin(1.5*ts*(2*pi));.25*sin(1.5*ts*(2*pi))])),desiredZMP);
      c = ZMPtracker(r,dZMP);
      output_select(1).system = 1;
      output_select(1).output = 1;
      sys = mimoFeedback(r,c,[],[],[],output_select);

      v = r.constructVisualizer();
      
      ytraj = sys.simulate(dZMP.tspan,randn(4,1));
      v.playback(ytraj);
    end
  end
  
  properties
    h
    g
  end
  
end
