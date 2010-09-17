classdef PendulumLQR < TILQRControl

  methods
    function obj = PendulumLQR(plant)
      typecheck(plant,'PendulumPlant');

      Q = diag([10,1]); R = 1;
      obj = obj@TILQRControl(plant,[pi;0],0,Q,R);
      obj = setWrapping(obj,[1,0],[-pi/2,3*pi/2;-inf,inf]);
      obj.control_dt = 0;
      
      if (checkDependency('spot_enabled'))
        options = struct('plant_order',3);
        obj = obj.verify(options);
      else
        warning('Since you don''t have SPOT, I''ll just use a reasonable guess for the region of attraction');
        obj.rho = 4;
      end
    end
  end
  
end
