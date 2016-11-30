classdef ForceTorque < LCMCoordinateFrame & Singleton
  
  methods
    function obj=ForceTorque()
      coordinates = {'fx','fy','fz','tx','ty','tz'};
      obj = obj@LCMCoordinateFrame('ForceTorque',drake.lcmt_force_torque,'f',coordinates);
      obj = obj@Singleton();
    end
  end
end
