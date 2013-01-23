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
      obj = setInputFrame(obj,CoordinateFrame('2DLIMPinput',1,'u',{'xddot_com'}));
      obj = setStateFrame(obj,CoordinateFrame('2DLIMPstate',2,'x',{'x_com','xdot_com'}));
      obj = setOutputFrame(obj,MultiCoordinateFrame({getStateFrame(obj),CoordinateFrame('2DLIMPoutput',1,'y',{'x_zmp'})}));
    end
    
    function v = constructVisualizer(obj)
      r = PlanarRigidBodyManipulator('CartTable.urdf',struct('floating',true,'view','right'));
      v{1} = r.constructVisualizer();
      try 
        addTransform(obj.getStateFrame,AffineTransform(obj.getStateFrame,v{1}.getInputFrame,sparse([4 8],[1 2],[1 1],8,2),zeros(8,1)));
      catch ex
        % it's possible that I already constructed the transform. that's ok.
        if ~strcmp(ex.identifier,'Drake:CoordinateFrame:ExistingTransform')
          rethrow(ex);
        end
      end
      zmp_point_fr = SingletonCoordinateFrame('ZMP2D',2,'z',{'x_zmp','z_zmp'});
      try 
        addTransform(getFrameByNum(obj.getOutputFrame,2),AffineTransform(getFrameByNum(obj.getOutputFrame,2),zmp_point_fr,sparse(1,1,1,2,1),zeros(2,1)));
      catch ex
        % it's possible that I already constructed the transform. that's ok.
        if ~strcmp(ex.identifier,'Drake:CoordinateFrame:ExistingTransform')
          rethrow(ex);
        end
      end
      v{2} = Point2DVisualizer(zmp_point_fr,'r*','MarkerSize',20,'LineWidth',3);
      v = MultiVisualizer(v);
    end
    
    function varargout = lqr(obj)
      % objective min_u \int dt [ x_zmp(t)^2 ] 
      Q = obj.Czmp'*obj.Czmp; R = obj.Dzmp'*obj.Dzmp;  options.N = obj.Czmp'*obj.Dzmp;
      varargout = cell(1,nargout);
      [varargout{:}] = tilqr(obj,Point(obj.getStateFrame),Point(obj.getInputFrame),Q,R,options);
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
      c = lqr(r);
      output_select(1).system = 1;
      output_select(1).output = 1;
      output_select(2).system = 1;
      output_select(2).output = 2;
      sys = mimoFeedback(r,c,[],[],[],output_select);
      
      v = r.constructVisualizer();

      for i=1:5;
        ytraj = sys.simulate([0 5],randn(2,1));
        v.playback(ytraj);
      end
    end
  end
  
end