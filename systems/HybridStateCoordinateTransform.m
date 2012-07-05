classdef HybridStateCoordinateTransform < CoordinateTransform
  
  methods
    function obj = HybridStateCoordinateTransform(hybrid_system,mode_system,mode_number)
      typecheck(hybrid_system,'HybridDrakeSystem');
      typecheck(mode_system,'DrakeSystem');
      typecheck(mode_number,'double'); 
      sizecheck(mode_number,[1 1]);
      obj = obj@CoordinateTransform(hybrid_system.getStateFrame,mode_system.getStateFrame,true,true);
      obj.hs_num_xd = hybrid_system.getNumDiscStates();
      obj.hs_num_xc = hybrid_system.getNumContStates();
      obj.ms_num_xd = mode_system.getNumDiscStates();
      obj.ms_num_xc = mode_system.getNumContStates();
      obj.mode_number = mode_number;
    end
    
    function xm = output(obj,~,~,x)
      if (x(1)~=obj.mode_number)
        error('Incorrect mode.  Cannot convert this hybrid state into the mode state'); 
      end
      ind = 1; xm=[];
      if (obj.hs_num_xd>0)
        xm = x(ind+(1:obj.ms_num_xd));
        ind = ind+obj.hs_num_xd;
      end
      if (obj.hs_num_xc>0)
        xm = [xm;x(ind+(1:obj.ms_num_xc))];
      end
    end
  end
  
  properties
    hs_num_xd;
    hs_num_xc;
    ms_nux_xd;
    ms_num_xc;
    mode_number;
  end
end
