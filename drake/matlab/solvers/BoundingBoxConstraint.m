classdef BoundingBoxConstraint < LinearConstraint
  % enforce a bounding box constraint lb<=x<=ub
  
  methods
    function obj = BoundingBoxConstraint(lb,ub)
      obj = obj@LinearConstraint(lb,ub,speye(numel(lb)));
    end
  end
end