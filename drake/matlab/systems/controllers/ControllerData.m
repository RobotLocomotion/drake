classdef ControllerData < SharedDataHandle
  % ControllerData is designed to be the parent of
  % classes that contain data shared between controller modules. It is a
  % handle class because some of these properties change during execution
  % and it is desirable for all modules' references to be updated.
  
  % optional: for properties that change infrequently or never
  properties (SetAccess=private,GetAccess=public)

  end
  
  % optional: for properties that can be modified 'on the fly'
  properties (SetAccess=public,GetAccess=public)
  end

  methods
    function obj=ControllerData(data)
      obj = obj@SharedDataHandle(data);
      verifyControllerData(obj,data);
      updateControllerData(obj,data);
    end
  end
  
  methods (Abstract)
    % asserts existence and type of properties contained in 'data'
    verifyControllerData(obj,data);
  end

  methods
    function updateControllerData(obj, data)
      % sets class properties using values contained in 'data'
      for f = fieldnames(data)'
        obj.(f{1}) = data.(f{1});
      end
    end
  end
end