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
      %   @param origin -- Point object representing the origin of the
      %                    hyperplane
      %   @param normal -- Point object representing the (possibly non-unit)
      %                    normal of the hyperplane. Must be in the same frame
      %                    as origin
      %
      %   @retval obj   -- drakeFunction.euclidean.SignedDistanceToHyperplane
      %                    object
      typecheck(origin,'Point');
      typecheck(normal,'Point');
      assert(origin.frame == normal.frame, ...
        'origin and normal must be expressed in the same coordinate frame.');
      input_frame = origin.frame;
      output_frame = drakeFunction.frames.realCoordinateSpace(1);
      n = reshape(normalizeVec(double(normal)),[],1);
      r = reshape(double(origin),[],1);
      A = n';
      b = -n'*r;
      obj = obj@drakeFunction.Affine(input_frame,output_frame,A,b);
      obj.origin = origin;
      obj.normal = Point(normal.frame,n);
    end
  end
end
