classdef Quasistatic < atlasParams.Base
  methods
    function obj = Quasistatic(r)
      typecheck(r, 'Atlas');
      obj = obj@atlasParams.Base(r);
    end
  end
end


