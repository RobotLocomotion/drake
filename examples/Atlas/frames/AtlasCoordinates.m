classdef AtlasCoordinates < LCMCoordinateFrameWCoder & Singleton
  % atlas q
  methods
    function obj=AtlasCoordinates(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      nq = r.getNumDOF();
      joint_names = r.getStateFrame.coordinates(1:nq); 
      coder = JointAnglesCoder('atlas',joint_names);
      obj = obj@LCMCoordinateFrameWCoder('AtlasCoordinates',nq,'x',JLCMCoder(coder));
      obj.setCoordinateNames(joint_names);
      obj.setDefaultChannel('DESIRED_ACCELERATION');
    end
  end
end
