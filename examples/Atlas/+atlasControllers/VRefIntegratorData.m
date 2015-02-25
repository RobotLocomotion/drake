classdef VRefIntegratorData < atlasControllers.IntegratorData
  properties
    fc_prev;
  end

  methods
    function obj = VRefIntegratorData(r)
      obj = obj@atlasControllers.IntegratorData(r);
      obj.fc_prev = struct('right', true, 'left', true);
    end
  end
end