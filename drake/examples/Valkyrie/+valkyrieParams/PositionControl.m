classdef PositionControl < valkyrieParams.Base
  methods
    function obj = PositionControl(r)
      typecheck(r, 'Valkyrie');
      obj = obj@valkyrieParams.Base(r);
      obj.whole_body.w_qdd = 0.001*ones(r.getNumVelocities(), 1);
      obj = obj.updateKd();
    end
  end
end



