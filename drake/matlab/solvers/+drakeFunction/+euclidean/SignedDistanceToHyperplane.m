classdef SignedDistanceToHyperplane < drakeFunction.Affine
  % DrakeFunction representing the distance from a point to a hyperplane
  properties (SetAccess = immutable)
    origin  % Point object representing the origin of the hyperplane
    normal  % Point object representing the unit normal of the hyperplane
  end

  methods
    function obj = SignedDistanceToHyperplane(origin,normal)
      % obj = SignedDistanceToHyperplane(origin,normal) returns a 
      %   SignedDistanceToHyperplane object 
      %
      %   @param origin -- n-element vector representing the origin of the
      %                    hyperplane
      %   @param normal -- n-element vector representing the (possibly non-unit)
      %                    normal of the hyperplane.
      %
      %   @retval obj   -- drakeFunction.euclidean.SignedDistanceToHyperplane
      %                    object
      sizecheck(normal, size(origin))
      n = reshape(normalizeVec(normal),[],1);
      r = reshape(origin,[],1);
      A = n';
      b = -n'*r;
      obj = obj@drakeFunction.Affine(A,b);
      obj.origin = origin;
      obj.normal = n;
    end
  end
end
