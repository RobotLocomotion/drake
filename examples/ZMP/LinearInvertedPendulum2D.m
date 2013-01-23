classdef LinearInvertedPendulum2D < LinearSystem
  % This is the two-dimensional model of the ZMP dynamics of the linear
  % inverted pendulum as described in Kajita03.
  %
  % - Here the input is the commanded linear *acceleration* of 
  % the center of mass (note that Kajita03 uses the jerk, but I don't think
  % it's warranted).  
  % - The state is [x,xdot]^T.  
  % - The output is the horizontal position of the ZMP.  
  
  properties
    Czmp
    Dzmp
  end
  
  methods 
    function obj = LinearInvertedPendulum2D(h,g)
      % @param h the (constant) height of the center of mass
      % @param g the scalar gravity @default 9.81

      typecheck(h,'numeric');
      rangecheck(h,0,inf);
      if (nargin<2) g=9.81; end
      Czmp = [1 0]; Dzmp = -h/g;
      obj = obj@LinearSystem([0 1; 0 0],[0; 1],[],[],[eye(2);Czmp],[0;0;Dzmp]);
      obj.Czmp = Czmp;
      obj.Dzmp = Dzmp;
      
      obj = setInputFrame(obj,CartTable2DInput);
      sframe = CoordinateFrame('LIMP2DState',2,'x',{'x_com','xdot_com'})
      addTransform(sframe,AffineTransform(sframe,CartTable2DState,sparse([4 8],[1 2],[1 1],8,2),zeros(8,1)));
      addTransform(CartTable2DState,AffineTransform(CartTable2DState,sframe,sparse([1 2],[4 8],[1 1],2,8),zeros(2,1)));
      obj = setStateFrame(obj,sframe);
      
      zmpframe = CoordinateFrame('ZMP1D',1,'z',{'x_zmp'});
      addTransform(zmpframe,AffineTransform(zmpframe,ZMP2D,sparse(1,1,1,2,1),zeros(2,1)));
      obj = setOutputFrame(obj,MultiCoordinateFrame({sframe,zmpframe}));
    end
    
    function v = constructVisualizer(obj)
      v = MultiVisualizer({CartTable2DVisualizer,ZMP2DVisualizer,desiredZMP2DVisualizer});
    end
    
    function varargout = lqr(obj)
      % objective min_u \int dt [ x_zmp(t)^2 ] 
      varargout = cell(1,nargout);
      [varargout{:}] = tilqry(obj,Point(obj.getStateFrame),Point(obj.getInputFrame),diag([0 0 1]),0);
    end
    
    function c = ZMPtracker(dZMP)
      valuecheck(getOutputFrame(dZMP),desiredZMP1D);
    end
  end
  
  methods (Static)
    function runPassive
      r = LinearInvertedPendulum2D(1.055);
      v = r.constructVisualizer();
      ytraj = r.simulate([0 3],[0;.1]);
      v.playback(ytraj);
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
      dZMP = setOutputFrame(FunctionHandleTrajectory(@(t) .08*sin(.125*t*(2*pi)),1,linspace(0,8,100)),desiredZMP1D);
      v = r.constructVisualizer();
      
      ytraj = r.simulate(dZMP.tspan,zeros(2,1));%randn(2,1));
      v.playback([ytraj;dZMP]);
    end
  end
  
end