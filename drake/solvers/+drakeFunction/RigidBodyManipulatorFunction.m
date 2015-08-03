classdef RigidBodyManipulatorFunction < drakeFunction.DrakeFunction & RigidBodyElement
  % Abstract parent class for functions that need to store a
  % RigidBodyManipulator
  properties (SetAccess = protected)
    rbm %RigidBodyManipulator object
  end
  methods
    function obj = RigidBodyManipulatorFunction(rbm, dim_input, dim_output)
      % obj = drakeFunction.RigidBodyManipulatorFunction(rbm,input_frame,output_frame)
      %
      % @param rbm          -- RigidBodyManipulator object
      % @param input_frame  -- CoordinateFrame object
      % @param output_frame -- CoordinateFrame object
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
      obj = setRigidBodyManipulator(obj,rbm);
    end

    function obj = setRigidBodyManipulator(obj, rbm)
      typecheck(rbm,{'RigidBodyManipulator', ...
                     'TimeSteppingRigidBodyManipulator'});
      obj.rbm = rbm.removeAllStateConstraints();  % so I don't end up with recursive copies
    end
  end
end

