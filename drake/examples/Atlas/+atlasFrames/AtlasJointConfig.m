classdef AtlasJointConfig < SingletonCoordinateFrame
  % atlas joint position frame (coordinate ordering from atlas state frame, 
  % not input frame)
  methods
    function obj=AtlasJointConfig(r,floating)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      if nargin>1
        typecheck(floating,'logical');
      else
        floating = true;
      end
      
      nq = r.getNumPositions();
      if floating
        jrange = 7:nq; % ignore floating base dofs
      else
        jrange = 1:nq;
      end
      
      coords = r.getStateFrame.getCoordinateNames();
      joint_names = coords(jrange); 
      obj = obj@SingletonCoordinateFrame('NominalPositionGoal',length(jrange),'x',joint_names);
    end
  end
end
