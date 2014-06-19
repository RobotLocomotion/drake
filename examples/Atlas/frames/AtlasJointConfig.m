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
      
      nq = r.getnumPositions();
      if floating
        jrange = vertcat(r.body(3:end).position_num); % ignore floating base dofs
      else
        jrange = 1:nq;
      end
      
      joint_names = r.getStateFrame.coordinates(jrange); 
      obj = obj@SingletonCoordinateFrame('NominalPositionGoal',length(jrange),'x',joint_names);
    end
  end
end
