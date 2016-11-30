classdef HybridStateTransform < CoordinateTransform
  
  methods
    function obj = HybridStateTransform(hybrid_system,mode_system,mode_number)
      typecheck(hybrid_system,'HybridDrakeSystem');
      typecheck(mode_system,'DrakeSystem');
      typecheck(mode_number,'double'); 
      if (~isvector(mode_number)) error('mode_number must be a vector'); end
      obj = obj@CoordinateTransform(hybrid_system.getStateFrame,mode_system.getStateFrame,true,true);
      obj.hs_num_xd = hybrid_system.getNumDiscStates();
      obj.hs_num_xc = hybrid_system.getNumContStates();
      obj.ms_num_xd = mode_system.getNumDiscStates();
      obj.ms_num_xc = mode_system.getNumContStates();
      obj.mode_number = mode_number;
    end
    
    function xm = output(obj,~,~,x)
      if ~any(x(1)==obj.mode_number)
        error('Incorrect mode.  Cannot convert this hybrid state into the mode state'); 
      end
      ind = 0; xm=[];
      if (obj.hs_num_xd>0)
        xm = x(ind+(1:obj.ms_num_xd));
        ind = ind+obj.hs_num_xd;
      end
      if (obj.hs_num_xc>0)
        xm = [xm;x(ind+(1:obj.ms_num_xc))];
      end
    end
    
    function tf = addModeNumber(tf,i)  
      if ~any(tf.mode_number==i)
        tf.mode_number = [tf.mode_number(:);i];
      end
    end
  end
  
  properties
    hs_num_xd;
    hs_num_xc;
    ms_num_xd;
    ms_num_xc;
    mode_number;  % can be a vector of allowed modes
  end
end
