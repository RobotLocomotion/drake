classdef BodySpatialAcceleration < CoordinateFrame
  % frame contains body index + 6 dim spatial acceleration of the body

  % NOTE: currently hacking this frame by sending nonspatial body
  % accelerations, thus the backwards ordering
  methods
    function obj=BodySpatialAcceleration(r,name)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      coordinates = {strcat(name,'_ind'),strcat(name,'_xdd'),strcat(name,'_ydd'),...
        strcat(name,'_zdd'),strcat(name,'_omdx'),strcat(name,'_omdy'),strcat(name,'_omdz')};
      obj = obj@CoordinateFrame(name,7,'x',coordinates);
    end
  end
end
