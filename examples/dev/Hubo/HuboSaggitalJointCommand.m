classdef HuboSaggitalJointCommand < LCMCoordinateFrame
  
  methods
    function obj = HuboSaggitalJointCommand
      obj = obj@LCMCoordinateFrame('HuboSaggitalJointCommand','lcmtypes.lcmt_hubo_saggital_jointcommand','d');
      fr = HuboJointCommand();
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