classdef KinematicMotionPlanningProblem < MotionPlanningProblem

  properties
    manip
    kinematics_constraints
  end

  methods
    function obj = KinematicMotionPlanningProblem(manip)
      typecheck(manip,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      
      nq = getNumPosition(manip);
      state_names = getCoordinateNames(getStateFrame(manip));

      obj = obj@MotionPlanningProblem(nq,state_names{1:nq});
      obj.manip = manip;
    end


    function obj = addKinematicConstraint(obj,constraint,varargin)

    end

  end

end
