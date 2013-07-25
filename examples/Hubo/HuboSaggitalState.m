classdef HuboSaggitalState < LCMCoordinateFrame
  
  methods
    function obj = HuboSaggitalState
      obj = obj@LCMCoordinateFrame('HuboSaggitalState','lcmtypes.lcmt_hubo_saggital_state','x');
      fr = HuboState();
      try 
        obj.addProjectionTransformByCoordinateNames(fr,0); % todo: add default out-of-plane joint coordinates here
        fr.addProjectionTransformByCoordinateNames(obj,0);
      catch ex
        if (strcmp(ex.identifier, 'Drake:CoordinateFrame:ExistingTransform'))
          % ok... if those projections exist, then no need to make them
          % again
        else
          rethrow(ex);
        end
      end
    end
  end
  
end