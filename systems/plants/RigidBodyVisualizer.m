classdef RigidBodyVisualizer < Visualizer
  properties (Access=protected)
    model;
  end
  methods
    function obj = RigidBodyVisualizer(manip)
      obj = obj@Visualizer(getStateFrame(manip));
      obj.model = manip;
    end
    function inspector(obj,x0, state_dims,minrange,maxrange)
      if (nargin<2), x0 = getInitialState(obj.model); end
      if (nargin<3), state_dims = 1:getNumDOF(obj.model); end
      if any(state_dims)>getNumDOF(obj.model)
        error('can''t draw velocities');
      end
      [jlmin,jlmax] = getJointLimits(obj.model);
      jlmin(isinf(jlmin))=-pi; jlmax(isinf(jlmax))=pi;
      if (nargin<4), minrange = jlmin(state_dims); end
      if (nargin<5), maxrange = jlmax(state_dims); end
      
      inspector@Visualizer(obj,x0,state_dims,minrange,maxrange);
    end
  end
end

