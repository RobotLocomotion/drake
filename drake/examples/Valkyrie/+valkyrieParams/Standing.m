classdef Standing < valkyrieParams.Base
  methods
    function obj = Standing(r)
      typecheck(r, 'Valkyrie');
      obj = obj@valkyrieParams.Base(r);
    end
  end
end


