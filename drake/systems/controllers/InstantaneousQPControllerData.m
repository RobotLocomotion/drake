classdef InstantaneousQPControllerData < ControllerData
  % Class that contains data needed by the QPController and cascaded modules.

  % properties that change infrequently or never
  properties (SetAccess=private,GetAccess=public)
  end
  
  % properties that can be modified 'on the fly'
  properties (SetAccess=public,GetAccess=public)
    % solver related -------------------------------------------------------------
    infocount=0 % number of consecutive iterations with solver info < 0
    qp_active_set=[]% active set of inequality constraints from pervious iteration
    num_active_contact_pts=0;
  end
  
  methods 
    function obj = InstantaneousQPControllerData(data)
      typecheck(data,'struct');
      obj = obj@ControllerData(data);
    end
 
    function data=verifyControllerData(~,data)
    end
        
  %   function updateControllerData(obj,data)
  %     updateControllerData@ControllerData(obj,data);
  %   end
  end
end