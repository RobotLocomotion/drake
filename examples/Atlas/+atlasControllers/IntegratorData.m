classdef IntegratorData < ControllerData
% Container class for integrator state
  properties
    state;
    t_prev;
  end

  methods
    function obj = IntegratorData(r)
      data = struct('state', zeros(r.getNumPositions(), 1),...
                    't_prev', nan);
      obj = obj@ControllerData(data);
    end

    function verifyControllerData(obj,data)
      assert(isscalar(data.t_prev));
      assert(isnumeric(data.state));
    end
  end

end