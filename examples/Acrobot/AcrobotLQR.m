classdef AcrobotLQR < TILQRControl

  methods
    function obj = AcrobotLQR(dyn)
      Q = diag([10,10,1,1]); R = 1;
      obj = obj@TILQRControl(dyn,[pi;0;0;0],0,Q,R);
      obj = setWrapping(obj,[1,1,0,0],[-pi/2,3*pi/2;-pi/2,3*pi/2;-inf,inf;-inf,inf]);
      obj.control_dt = 0;
      
      checkDependency('spot_enabled');
      options = struct('plant_order',3);
      obj = obj.verify(options);
    end
  end
  
end
