classdef Standing < atlasParams.Base
  methods
    function obj = Standing(r)
      typecheck(r, 'Atlas');
      obj = obj@atlasParams.Base(r);
    end
  end
end


