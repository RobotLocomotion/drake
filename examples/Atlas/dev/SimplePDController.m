classdef SimplePDController < MIMODrakeSystem
  % outputs a desired q_ddot (including floating dofs)
  methods
    function obj = SimplePDController(r)
      typecheck(r,'Atlas');
      
      coords = AtlasCoordinates(r);
      input_frame = MultiCoordinateFrame({coords,r.getStateFrame});
      output_frame = coords;

      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.manip = r;
    end
   
  	function y=mimoOutput(obj,t,~,varargin)
      q_des = varargin{1};
      x = varargin{2};
      nq = getNumDOF(obj.manip);
      q = x(1:nq);
      qd = x(nq+1:end);

%       disp('simple pd');
%       x
      
%     [Kp_q,Kd_q] = getQPpdGains(obj.manip);
       Kp_q = 200*eye(nq);
       Kd_q = 20.0*eye(nq);
%       Kp_q(1:6,1:6) = zeros(6); % ignore free body dofs
%       Kd_q(1:6,1:6) = zeros(6); % ignore free body dofs
       Kp_q(1:2,1:2) = zeros(2); % ignore x,y
       Kd_q(1:2,1:2) = zeros(2); % ignore x,y
      err_q = q_des - q;
      y = Kp_q*err_q - Kd_q*qd;
    end
  end
  
  properties
    manip
    qstar
  end
end
