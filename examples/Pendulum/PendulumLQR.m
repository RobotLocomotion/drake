classdef PendulumLQR < TILQRControl

  methods
    function obj = PendulumLQR(dyn)
      Q = diag([10,1]); R = 1;
      obj = obj@TILQRControl(dyn,[pi;0],0,Q,R);
      obj = setWrapping(obj,[1,0],[-pi/2,3*pi/2;-inf,inf]);
      obj.control_dt = 0;
      
      options = struct('plant_order',3);
      obj = obj.verify(options);
    end
  end
  
end
