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
      input_frame = rbm.getPositionFrame();
      obj = obj@drakeFunction.RigidBodyManipulatorFunction(rbm, ...
        input_frame, output_frame);
      obj = obj.setSparsityPattern();
    end

    function obj = setSparsityPattern(obj)
      if isempty(obj.rbm)
        obj = setSparsityPattern@drakeFunction.RigidBodyManipulatorFunction(obj);
      else
        joint_idx = obj.kinematicsPathJoints();
        obj.iCfun = reshape(bsxfun(@times,(1:obj.getNumOutputs())',ones(1,length(joint_idx))),[],1);
        obj.jCvar = reshape(bsxfun(@times,ones(obj.getNumOutputs(),1),joint_idx),[],1);
      end
    end

    function obj = setRigidBodyManipulator(obj, rbm)
      obj = setRigidBodyManipulator@drakeFunction.RigidBodyManipulatorFunction(obj, rbm);
      assert(isequal_modulo_transforms(obj.input_frame,rbm.getPositionFrame()),...
        ['The input frame of this function is incompatible with the ', ...
         'position frame of the RigidBodyManipulator, rbm.']);
      obj.input_frame = rbm.getPositionFrame();
    end
  end

  methods (Access = protected)
    function joint_idx = kinematicsPathJoints(obj)
      % joint_idx = kinematicsPathJoints(obj) return the indices of the
      % joints used to evaluate the constraint. Should be overloaded by
      % child classes.
      %
      % @param obj          -- drakeFunction.kinematic.Kinematic object
      %
      % @retval joint_idx   -- Indices of the joints involved in this
      %                        constraint. Optional
      %                        @default (1:obj.robot.getNumPositions());
      joint_idx = (1:obj.rbm.getNumPositions());
    end
  end
end

