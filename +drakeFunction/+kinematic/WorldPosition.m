function fcn = WorldPosition(rbm,frame,varargin)
  fcn = drakeFunction.kinematic.RelativePosition(rbm,frame,1,varargin{:});
end
