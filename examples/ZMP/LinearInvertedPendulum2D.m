classdef LinearInvertedPendulum2D < LinearSystem
  % This is the two-dimensional model of the ZMP dynamics of the linear
  % inverted pendulum as described in Kajita03.
  %
  % - Here the input is the commanded linear *acceleration* of 
  % the center of mass (note that Kajita03 uses the jerk, but I don't think
  % it's warranted).  
  % - The state is [x,xdot]^T.  
  % - The output is the horizontal position of the ZMP.  
  
  methods 
    function obj = LinearInvertedPendulum2D(h,g)
      % @param h the (constant) height of the center of mass
      % @param g the scalar gravity @default 9.81
      typecheck(h,'numeric');
      rangecheck(h,0,inf);
      if (nargin<2) g=9.81; end
      Czmp = [1 0]; Dzmp = -h/g;
      obj = obj@LinearSystem([0 1; 0 0],[0; 1],[],[],[eye(2);Czmp],[0;0;Dzmp]);
      
      cartstateframe = CartTable2DState;
      obj = setInputFrame(obj,CartTable2DInput);
      sframe = SingletonCoordinateFrame('LIMP2DState',2,'x',{'x_com','xdot_com'});
      if isempty(findTransform(sframe,cartstateframe))
        addTransform(sframe,AffineTransform(sframe,cartstateframe,sparse([4 8],[1 2],[1 1],8,2),zeros(8,1)));
        addTransform(cartstateframe,AffineTransform(cartstateframe,sframe,sparse([1 2],[4 8],[1 1],2,8),zeros(2,1)));
      end
      obj = setStateFrame(obj,sframe);
      
      zmpframe = CoordinateFrame('ZMP1D',1,'z',{'x_zmp'});
      addTransform(zmpframe,AffineTransform(zmpframe,ZMP2D,sparse(1,1,1,2,1),zeros(2,1)));
      obj = setOutputFrame(obj,MultiCoordinateFrame({sframe,zmpframe}));
    end
    
    function v = constructVisualizer(obj)
      v = MultiVisualizer({CartTable2DVisualizer,ZMP2DVisualizer,desiredZMP2DVisualizer});
    end
    
    function varargout = lqr(obj,com0)
      % objective min_u \int dt [ x_zmp(t)^2 ] 
      varargout = cell(1,nargout);
      options.Qy = diag([0 0 1]);
      if (nargin<2) com0 = 0; end
      [varargout{:}] = tilqr(obj,Point(obj.getStateFrame,[com0;0*com0]),Point(obj.getInputFrame),zeros(2),0,options);
    end
    
    function c = ZMPtracker(obj,dZMP)
      typecheck(dZMP,'Trajectory');
      dZMP = dZMP.inFrame(desiredZMP1D);
      dZMP = setOutputFrame(dZMP,getFrameByNum(getOutputFrame(obj),2)); % override it before sending in to tvlqr
      
      zmp_tf = dZMP.eval(dZMP.tspan(end));
      [~,V] = lqr(obj,zmp_tf);

      options.tspan = linspace(dZMP.tspan(1),dZMP.tspan(end),10);
      options.sqrtmethod = false;
      x0traj = setOutputFrame(ConstantTrajectory([zmp_tf;0]),getStateFrame(obj));
      u0traj = setOutputFrame(ConstantTrajectory(0),getInputFrame(obj));
      options.Qy = diag([0,0,1]);
      options.ydtraj = [x0traj;dZMP];
      S = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
      c = tvlqr(obj,x0traj,u0traj,zeros(2),0,V,options);
      warning(S);
    end
    
    
    function comtraj = ZMPplanFromTracker(obj,com0,comdot0,dZMP,c)  
      % get a com plan from the ZMP tracking controller
      doubleIntegrator = LinearSystem([0 1;0 0],[0;1],[],[],eye(2),[0;0]);
      doubleIntegrator = setInputFrame(doubleIntegrator,getInputFrame(obj));
      doubleIntegrator = setStateFrame(doubleIntegrator,getStateFrame(obj));
      doubleIntegrator = setOutputFrame(doubleIntegrator,getStateFrame(obj));  % looks like a typo, but this is intentional: output is only the LIMP state
      sys = feedback(doubleIntegrator,c);
      
      comtraj = simulate(sys,dZMP.tspan,[com0;comdot0]);
      comtraj = inOutputFrame(comtraj,sys.getOutputFrame);
      comtraj = comtraj(1);  % only return position (not velocity)
    end
    
    function comtraj = ZMPplan(obj,com0,comdot0,dZMP)
      sizecheck(com0,1);
      sizecheck(comdot0,1);
      typecheck(dZMP,'Trajectory');
      dZMP = dZMP.inFrame(desiredZMP1D);

      
    end
    
    function comtraj = ZMPPlanner(obj,com0,comdot0,dZMP,options)
      warning('Drake:DeprecatedMethod','The ZMPPlanner function is deprecated.  Please use ZMPPlan (using the closed-form solution) or ZMPPlanFromTracker'); 
    
      % got a com plan from the ZMP tracking controller
      if(nargin<5) options = struct(); end
      c = ZMPtracker(obj,dZMP,options);
      comtraj = ZMPPlanFromTracker(obj,com0,comdot0,dZMP,c);
    end
    
    
  end
  
  methods (Static)
    function runPassive
      r = LinearInvertedPendulum2D(1.055);
      v = r.constructVisualizer();
      dZMP = setOutputFrame(ConstantTrajectory([0]),desiredZMP1D); 
      ytraj = r.simulate([0 3],[0;.1]);
      v.playback([ytraj;dZMP]);
    end
    
    function runLQR
      r = LinearInvertedPendulum2D(1.055);
      dZMP = setOutputFrame(ConstantTrajectory([0]),desiredZMP1D); 
      c = lqr(r);
      output_select(1).system = 1;
      output_select(1).output = 1;
      output_select(2).system = 1;
      output_select(2).output = 2;
      sys = mimoFeedback(r,c,[],[],[],output_select);
      
      v = r.constructVisualizer();

      for i=1:5;
        ytraj = sys.simulate([0 5],randn(2,1));
        v.playback([ytraj;dZMP]);
      end
    end
    
    function ZMPtrackingDemo()
      r = LinearInvertedPendulum2D(1.055);
      ts = linspace(0,8,100); 
      dZMP = setOutputFrame(PPTrajectory(spline(ts,.08*sin(1.5*ts*(2*pi)))),desiredZMP1D);
      c = ZMPtracker(r,dZMP);
      output_select(1).system = 1;
      output_select(1).output = 1;
      output_select(2).system = 1;
      output_select(2).output = 2;
      sys = mimoFeedback(r,c,[],[],[],output_select);

      v = r.constructVisualizer();
      
      ytraj = sys.simulate(dZMP.tspan,randn(2,1));
      v.playback([ytraj;dZMP]);
    end
  end
  
end