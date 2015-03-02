classdef Kinematic < atlasParams.Base
  methods
    function obj = Kinematic(r)
      typecheck(r, 'Atlas');
      obj = obj@atlasParams.Base(r);
      obj.whole_body.w_qdd = 0.001*ones(r.getNumVelocities(), 1);
      obj = obj.updateKd();
    end
  end
end



