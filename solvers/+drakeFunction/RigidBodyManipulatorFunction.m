classdef RigidBodyManipulatorFunction < drakeFunction.DrakeFunction
  % Abstract parent class for functions that need to store a
  % RigidBodyManipulator
  properties (SetAccess = protected)
    rbm %RigidBodyManipulator object
  end
  methods
    function obj = RigidBodyManipulatorFunction(rbm,input_frame,output_frame)
      % obj = drakeFunction.RigidBodyManipulatorFunction(rbm,input_frame,output_frame)
      %
      % @param rbm          -- RigidBodyManipulator object
      % @param input_frame  -- CoordinateFrame object
      % @param output_frame -- CoordinateFrame object
      typecheck(rbm,{'RigidBodyManipulator', ...
                     'TimeSteppingRigidBodyManipulator'});
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.rbm = rbm.removeAllStateConstraints();
    end

    function obj = setRigidBodyManipulator(obj, rbm)
      typecheck(rbm,{'RigidBodyManipulator', ...
                     'TimeSteppingRigidBodyManipulator'});
      obj.rbm = rbm;
    end
  end
end

