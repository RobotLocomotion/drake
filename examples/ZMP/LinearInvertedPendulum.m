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
        Dzmp = -h/g*eye(2);
        D = [zeros(4,2);Dzmp];
      elseif(isa(h,'PPTrajectory'))
        Czmp = [eye(2),zeros(2)];
        h_breaks = h.getBreaks;
        ddh = fnder(h,2);
        Dzmp_val = -h.eval(h_breaks)./(g+ddh.eval(h_breaks));
        D_val = zeros(6,2,length(h_breaks));
        D_val(5,1,:) = Dzmp_val; D_val(6,2,:) = Dzmp_val;
        D = PPTrajectory(spline(h_breaks,D_val));
      else
        error('LinearInvertedPendulum: unknown type for argument h.')
      end
             
      obj = obj@LinearSystem([zeros(2),eye(2);zeros(2,4)],[zeros(2);eye(2)],[],[],[eye(4);Czmp],D);
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
      
      typecheck(dZMP,'Trajectory');
      dZMP = dZMP.inFrame(desiredZMP);
      dZMP = setOutputFrame(dZMP,getFrameByNum(getOutputFrame(obj),2)); % override it before sending in to tvlqr
      
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
      % note: the system is actually linear, so x0traj and u0traj should
      % have no effect on the numerical results (they just set up the
      % frames)
      x0traj = setOutputFrame(ConstantTrajectory([zmp_tf;0;0]),obj.getStateFrame);
      u0traj = setOutputFrame(ConstantTrajectory([0;0]),obj.getInputFrame);
      options.Qy = diag([0 0 0 0 1 1]);
      options.ydtraj = [x0traj;dZMP];
      S = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
      [c,Vt] = tvlqr(obj,x0traj,u0traj,Q,zeros(2),V,options);
      warning(S);
    end
    
    function comtraj = ZMPplanFromTracker(obj,com0,comdot0,dZMP,c)
      doubleIntegrator = LinearSystem([zeros(2),eye(2);zeros(2,4)],[zeros(2);eye(2)],[],[],eye(4),zeros(4,2));
      doubleIntegrator = setInputFrame(doubleIntegrator,getInputFrame(obj));
      doubleIntegrator = setStateFrame(doubleIntegrator,getStateFrame(obj));
      doubleIntegrator = setOutputFrame(doubleIntegrator,getStateFrame(obj));  % looks like a typo, but this is intentional: output is only the LIMP state
      sys = feedback(doubleIntegrator,c);
      
      comtraj = simulate(sys,dZMP.tspan,[com0;comdot0]);
      comtraj = inOutputFrame(comtraj,sys.getOutputFrame);
      comtraj = comtraj(1:2);  % only return position (not velocity)
    end
    
    function comtraj = ZMPplan(obj,com0,comf,dZMP,options)
      % implements analytic method from Harada06
      %  notice the different arugments (comf instead of comdot0)
      sizecheck(com0,2);
      sizecheck(comf,2);
      typecheck(dZMP,'PPTrajectory');  
      dZMP = dZMP.inFrame(desiredZMP);
      if ~isnumeric(obj.h) error('variable height not implemented yet'); end
      
      Tc = sqrt(obj.g/obj.h);
      for ax=1:2
        ax_zmp = dZMP(ax);
        [breaks,coefs,l,k,d] = unmkpp(ax_zmp.pp);
        m = l; %  to match Harada06 eq (1)
        n = k-1;
        assert(prod(d)==1);
      
        % equations (6) and (7), solved for A(i+1,j)
        A(n+1,:) = coefs(:,1)';
        A(n,:) = coefs(:,2)';
        for i=n-2:-1:0
          A(i+1,:) = coefs(:,n+1-i)' + (obj.h/obj.g)*(i+1)*(i+2)*A(i+3,:);
        end

        % equation (12)
        dt = diff(breaks);
        dtn = ones(1,m); for i=1:n, dtn(i+1,:) = dt.*dtn(i,:); end  % dtn(a,j) = dt(j)^(a-1)
        w = [com0(ax) - A(1,1); ...
          reshape([A(1,2:end) - sum(A(:,1:end-1).*dtn(:,1:end-1)); ...
          A(2,2:end) - sum(repmat((1:n)',1,m-1).*A(2:end,1:end-1).*dtn(1:end-1,1:end-1))],2*(m-1),1); ...
          comf(ax) - A(:,end)'*dtn(:,end) ];
        
        Z = zeros(2*m);
        Z(1,1) = 1;
        % todo: vectorize this.  (should it be a sparse matrix?)
        for j=1:m-1,
          Z(2*j+(0:1),2*(j-1) + (1:4)) = [cosh(Tc*dt(j)),    sinh(Tc*dt(j)),   -1, 0; ...
                                        Tc*sinh(Tc*dt(j)), Tc*cosh(Tc*dt(j)), 0, -Tc];
        end
        Z(end,end-1:end) = [ cosh(Tc*dt(m)), sinh(Tc*dt(m)) ];
      
        y = reshape((Z\w)',2,m);
        V = y(1,:); W = y(2,:);
      
        % equation (5)
        % Note: i could return a function handle trajectory which is exact
        % (with the cosh,sinh terms), but for now I think it's sufficient and more
        % practical to make a new spline.
        com_knots(ax,:) = [V+A(1,:),V(end)*cosh(Tc*dt(end)) + W(end)*sinh(Tc*dt(end)) + A(:,end)'*dtn(:,end)];
      end
      comtraj = PPTrajectory(spline(breaks,com_knots));
    end
    
    function comtraj = ZMPplanner(obj,com0,comdot0,dZMP,options)
      warning('Drake:DeprecatedMethod','The ZMPPlanner function is deprecated.  Please use ZMPPlan (using the closed-form solution) or ZMPPlanFromTracker'); 
    
      % got a com plan from the ZMP tracking controller
      if(nargin<5) options = struct(); end
      c = ZMPtracker(obj,dZMP,options);
      comtraj = ZMPPlanFromTracker(obj,com0,comdot0,dZMP,c);
    end
    
    
  end
  
  methods (Static)
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
