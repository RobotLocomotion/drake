classdef Concatenated < drakeFunction.Container
  methods
    function obj = Concatenated(varargin)
      obj = obj@drakeFunction.Container(varargin{:});
    end
    function [f,df] = combineOutputs(obj,f_cell,df_cell)
      f = vertcat(f_cell{:});
      if obj.same_input
        df = vertcat(df_cell{:});
      else
        df = blkdiag(df_cell{:});
      end
    end
  end
end
