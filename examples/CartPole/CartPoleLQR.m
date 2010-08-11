classdef CartPoleLQR < TILQRControl

  methods
    function obj = CartPoleLQR(dyn)
      typecheck(dyn,'CartPoleDynamics');
      
      Q = diag([100,10,1,1]); R = 10;
      xG = [0;pi;0;0]; uG = 0;
      obj = obj@TILQRControl(dyn,xG,uG,Q,R);
      obj = setWrapping(obj,[0,1,0,0],[-inf,inf;0,2*pi;-inf,inf;-inf,inf]);
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
