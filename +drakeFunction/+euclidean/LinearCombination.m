classdef LinearCombination < drakeFunction.DrakeFunction
  properties (SetAccess = protected)
    n_pts
    dim_pts
  end
  methods
    function obj = LinearCombination(n_pts,frame)
      weights_frame = drakeFunction.frames.R(n_pts);
      pts_frame = MultiCoordinateFrame.constructFrame(repmat({frame},1,n_pts));
      input_frame = MultiCoordinateFrame({pts_frame,weights_frame});
      output_frame = frame;
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.dim_pts = frame.dim;
      obj.n_pts = n_pts;
    end
    function [f,df] = eval(obj,x)
      pts = reshape(x(1:obj.n_pts*obj.dim_pts),obj.dim_pts,obj.n_pts);
      weights = reshape(x(obj.n_pts*obj.dim_pts+(1:obj.n_pts)),[],1);
      f = pts*weights;
      df = zeros(obj.dim_pts,obj.n_pts+obj.n_pts*obj.dim_pts);
      for i = 1:obj.n_pts
        df(:,(i-1)*obj.dim_pts+(1:obj.dim_pts)) = weights(i)*eye(obj.dim_pts);
      end
      df(:,obj.n_pts*obj.dim_pts+(1:obj.n_pts)) = pts;
    end
  end
end
