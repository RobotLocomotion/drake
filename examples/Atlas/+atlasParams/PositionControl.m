classdef PositionControl < atlasParams.Base
  methods
    function obj = PositionControl(r)
      typecheck(r, 'Atlas');
      obj = obj@atlasParams.Base(r);
      obj.whole_body.w_qdd = 0.001*ones(r.getNumVelocities(), 1);
      obj = obj.updateKd();
    end
  end
end



