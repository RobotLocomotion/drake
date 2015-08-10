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
        C = [eye(4);Czmp];
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
    
    function varargout = lqr(obj,com0,Qy)
      % objective min_u \int dt [ x_zmp(t)^2 ] 
      varargout = cell(1,nargout);
      if nargin>2
        options.Qy = Qy;
      else
        options.Qy = diag([0 0 0 0 1 1]);
      end
      if (nargin<2) com0 = [0;0]; end
      [varargout{:}] = tilqr(obj,Point(obj.getStateFrame,[com0;0*com0]),Point(obj.getInputFrame),zeros(4),zeros(2),options);
    end
    
    function [ct,Vt,comtraj] = ZMPtracker(obj,dZMP,options)
      if nargin<3 options = struct(); end
      if ~isfield(options,'use_tvlqr') options.use_tvlqr = true; end
      if ~isfield(options,'compute_lyapunov') options.compute_lyapunov = (nargout>1); end
      if ~isfield(options,'Qy') options.Qy = diag([0 0 0 0 1 1]); end
      if ~options.use_tvlqr
        if isfield(options,'dCOM') || ~isTI(obj) || ~isa(dZMP,'PPTrajectory')
          warning('closed-form solution not implemented for these options (yet)');
          options.use_tvlqr = true;
        end
      end        

      if ~options.use_tvlqr
        assert(obj.g==9.81);
        
        if (nargout>2)
          [ct,Vt,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(obj.h,dZMP,options);
        elseif (nargout>1)
          [ct,Vt] = LinearInvertedPendulum.ZMPtrackerClosedForm(obj.h,dZMP,options);
        else
          ct = LinearInvertedPendulum.ZMPtrackerClosedForm(obj.h,dZMP,options);
        end
        
        zmp_tf = double(inFrame(Point(getOutputFrame(dZMP),dZMP.eval(dZMP.tspan(end))),desiredZMP));
        
        % set up frames
        ct = setInputFrame(ct,CoordinateFrame('com_tf',4,'x'));
        obj.getStateFrame.addTransform(AffineTransform(obj.getStateFrame,ct.getInputFrame,eye(4),-[zmp_tf;0;0]),true);
        ct.getInputFrame.addTransform(AffineTransform(ct.getInputFrame,obj.getStateFrame,eye(4),+[zmp_tf;0;0]),true);
        ct = setOutputFrame(ct,getInputFrame(obj));
        
        if (nargout>1)
          Vt = setFrame(Vt,getInputFrame(ct));
        end
      else
        typecheck(dZMP,'Trajectory');
        dZMP = dZMP.inFrame(desiredZMP);
        
        zmp_tf = dZMP.eval(dZMP.tspan(end));

        if(isTI(obj))
          [c,V] = lqr(obj,zmp_tf,options.Qy);
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
          [c,V] = tilqr(ti_obj,Point(ti_obj.getStateFrame,[zmp_tf;0*zmp_tf]),Point(ti_obj.getInputFrame),Q,zeros(2),options);
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
        
        dZMP = setOutputFrame(dZMP,getFrameByNum(getOutputFrame(obj),2)); % override it before sending in to tvlqr
        % note: the system is actually linear, so x0traj and u0traj should
        % have no effect on the numerical results (they just set up the
        % frames)
        x0traj = setOutputFrame(ConstantTrajectory([zmp_tf;0;0]),obj.getStateFrame);
        u0traj = setOutputFrame(ConstantTrajectory([0;0]),obj.getInputFrame);
        options.ydtraj = [x0traj;dZMP];
        WS = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
        [ct,Vt] = tvlqr(obj,x0traj,u0traj,Q,zeros(2),V,options);
        warning(WS);
        
        if (nargout>2)
          if ~isfield(options,'com0'), options.com0 = zeros(2,1); end
          if ~isfield(options,'comdot0'), options.comdot0 = zeros(2,1); end
          
          comtraj = COMplanFromTracker(obj,options.com0,options.comdot0,dZMP.tspan,ct);
        end
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
      c = ZMPtracker(obj,dZMP,options);
      comtraj = COMplanFromTracker(obj,com0,comdot0,dZMP.tspan,c);
    end
    
    
  end
  
  methods (Static)
    function [A, B, C, D, Q, R, Q1, R1, N] = setupLinearSystem(h_over_g, Qy)
      A = [zeros(2),eye(2);zeros(2,4)]; 
      B = [zeros(2);eye(2)];
      C = [eye(2), zeros(2)];
      D = -h_over_g*eye(2);

      % LQR costs for the output system
      Q = Qy(5:6,5:6); % for consistency with the other solution methods
      R = zeros(2);

      % Convert to costs in xbar, u
      Q1 = C'*Q*C;
      R1 = R + D'*Q*D;
      N = C'*Q*D;
    end

    function [ct,Vt,comtraj] = ZMPtrackerClosedForm(h,dZMP,options)
      if nargin<3 options = struct(); end
      options = applyDefaults(options, struct('compute_lyapunov', (nargout>1),...
                                              'Qy', diag([0,0,0,0,1,1]),...
                                              'use_lqr_cache', false,...
                                              'lqr_cache_com_height_resolution', 0.01,...
                                              'build_control_objects', true));
      if options.use_lqr_cache
        % round our CoM height value to improve the likelihood of a cache hit
        h = round(h / options.lqr_cache_com_height_resolution) * options.lqr_cache_com_height_resolution;
      end
      typecheck(dZMP,'Trajectory');
      dZMP = dZMP.inFrame(desiredZMP);
      
      zmp_tf = dZMP.eval(dZMP.tspan(end));

      % closed-form solution, see derivation in zmp_riccati.pdf
      [breaks,coefs,n,k,d] = unmkpp(dZMP.pp);
      assert(prod(d)==2);
      coefs_flipped = reshape(coefs,[2,n,k]);

      % matlab writes coefficients in descending order, we use ascending
      if logical(exist('flip','builtin')) % flipdim is deprecated, but flip does not exist in 2012b
        c = flip(coefs_flipped, 3); 
      else
        c = flipdim(coefs_flipped, 3);
      end

      c(:,:,1) = c(:,:,1) - repmat(zmp_tf,1,n);  % switch to zbar coordinates
      dt = diff(breaks);

      hg = h/9.81;
      [A, B, C, D, Q, R, Q1, R1, N] = LinearInvertedPendulum.setupLinearSystem(hg, options.Qy);
      R1i = inv(R1);
      
      if options.use_lqr_cache
        [K, S] = ZMPCachedLQR(hg, options.Qy);
      else
        [K,S] = lqr(A,B,Q1,R1,N); 
      end

      K=-K;
      
      NB = (N' + B'*S);
      A2 = NB'*R1i*B' - A';
      B2 = 2*(C' - NB'*R1i*D)*Q;
      
      assert(rank(A2)==4);
      A2i = inv(A2);
      
      alpha = zeros(4,n);
      beta = zeros(4,n,k);
      gamma = zeros(2,n,k);
      for j=n:-1:1
        beta(:,j,k) = -A2i*B2*c(:,j,k);
        gamma(:,j,k) = R1i*D*Q*c(:,j,k) - .5*R1i*B'*beta(:,j,k);
        for i=k-1:-1:1
          beta(:,j,i) = A2i*( i*beta(:,j,i+1) - B2*c(:,j,i) );
          gamma(:,j,i) = R1i*D*Q*c(:,j,i) - .5*R1i*B'*beta(:,j,i);
        end
        if (j==n)
          s1dt = zeros(4,1);
        else
          s1dt = alpha(:,j+1) + beta(:,j+1,1);
        end
        alpha(:,j) = expm(A2*dt(j)) \ (s1dt - squeeze(beta(:,j,:))*(dt(j).^(0:k-1)'));
      end
      
      if options.build_control_objects
        ct = AffineSystem([],[],[],[],[],[],[],K,ExpPlusPPTrajectory(breaks,-.5*R1i*B',A2,alpha,gamma));
      else
        ct = [];
      end
        
      if options.compute_lyapunov
        if options.build_control_objects
          s1traj = ExpPlusPPTrajectory(breaks,eye(4),A2,alpha,beta);
          [t,y,ydot] = ode4(@s2dynamics,fliplr(breaks),0);
          s2traj = PPTrajectory(pchipDeriv(breaks,fliplr(y.'),fliplr(ydot.')));
          Vt = QuadraticLyapunovFunction(getInputFrame(ct),S,s1traj,s2traj);
        else
          s1traj = ExpPlusPPTrajectory(breaks,eye(4),A2,alpha,beta);
          Vt = struct('S', S, 's1', s1traj);
        end
      else
        Vt = [];
      end
        
      if (nargout>2)
        Ay = [A + B*K, -.5*B*R1i*B'; zeros(4), A2];
        Ayi = inv(Ay);
        By = [B*R1i*D*Q; B2];
        
        a = zeros(4,n);
        b = zeros(4,n,k);
        
        x = zeros(4,1);
        if isfield(options,'com0'), x(1:2) = options.com0 - zmp_tf; end
        if isfield(options,'comdot0'), x(3:4) = options.comdot0; end
        
        for j=1:n
          b(:,j,k) = -Ayi(1:4,:)*By*c(:,j,k);
          for i=k-1:-1:1
            b(:,j,i) = Ayi(1:4,:)*( i*[b(:,j,i+1);beta(:,j,i+1)] - By*c(:,j,i) );
          end
          a(:,j) = x - b(:,j,1);
          x = [eye(4),zeros(4)]*expm(Ay*dt(j))*[a(:,j);alpha(:,j)] + squeeze(b(:,j,:))*(dt(j).^(0:k-1)');
          b(1:2,j,1) = b(1:2,j,1)+zmp_tf;  % back in world coordinates
        end
        
        comtraj = ExpPlusPPTrajectory(breaks,[eye(2),zeros(2,6)],Ay,[a;alpha],b(1:2,:,:));
      end
      
      function s2dot = s2dynamics(t,s2)
        [s1,j] = eval(s1traj,t);
        trel = t-breaks(j);
        zbar = squeeze(c(:,j,:))*trel.^(0:k-1)';
        r2 = -2*D*Q*zbar;
        rs = 0.5*(r2 + B'*s1);
        q3 = zbar'*Q*zbar;
        s2dot = -q3 + rs'*R1i*rs;
      end
    end
    
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
