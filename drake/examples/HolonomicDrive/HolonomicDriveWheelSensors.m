classdef HolonomicDriveWheelSensors < DrakeSystem
  % A system that produces wheel angles from output of a HolonomicDrive
  % This can be used to produce more detailed renderings
  properties
    drive
  end

  methods
    function obj = HolonomicDriveWheelSensors(drive)
      n = length(drive.wheels);
      obj = obj@DrakeSystem(n,0,6,n,false,true);
      obj.drive = drive;

      obj = setInputFrame(obj, drive.getOutputFrame);
      obj = setStateFrame(obj, CoordinateFrame('HolonomicRotorState',n,'u',...
        arrayfun(@(i) sprintf('phiu%d', i), 1:n, 'Unif', false)));
      obj = setOutputFrame(obj, obj.getStateFrame);
    end

    function x = getInitialState(obj)
      x = zeros(length(obj.drive.wheels), 1);
    end

    function xd = dynamics(obj,t,x,u)
      theta = u(3);
      rotation = rotmat(theta);

      bodyvel = rotation' * u(4:5);

      xd = obj.drive.rotorSpeeds(bodyvel, u(6));
    end

    function y = output(obj,t,x,u)
      y = x;
    end
  end
end
