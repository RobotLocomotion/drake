classdef VRefIntegratorData < atlasControllers.IntegratorData
% Integrator state for the atlas velocity reference
  properties
    fc_prev; % foot contact state from last controller tick
  end

  methods
    function obj = VRefIntegratorData(r)
      obj = obj@atlasControllers.IntegratorData(r);
      obj.fc_prev = struct('right', true, 'left', true);
    end
  end
end