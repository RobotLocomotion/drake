classdef Kinematic < drakeFunction.RigidBodyManipulatorFunction
  % Abstract parent class for kinematic functions
  %   All children of this class take the joint-angles of a specific
  %   RigidBodyManipulator as their input.
  methods
    function obj = Kinematic(rbm,output_frame)
      % @param rbm            -- [TimeStepping]RigidBodyManipulator
      %                          object
      % @param output_frame   -- CoordinateFrame object
      %
      % @retval obj   -- drakeFunction.kinematic.Kinematic object
      typecheck(rbm,{'RigidBodyManipulator', ...
                     'TimeSteppingRigidBodyManipulator'});
      input_frame = CoordinateFrame('position_frame', ...
        rbm.getNumPositions(),'q');
      obj = obj@drakeFunction.RigidBodyManipulatorFunction(rbm, ...
        input_frame, output_frame);
    end
  end
end

