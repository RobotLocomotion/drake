classdef SignedDistanceToHyperplane < drakeFunction.Affine
  methods
    function obj = SignedDistanceToHyperplane(origin,normal)
      typecheck(origin,'Point');
      typecheck(normal,'Point');
      assert(origin.frame == normal.frame, ...
        'origin and normal must be expressed in the same coordinate frame.');
      input_frame = origin.frame;
      output_frame = drakeFunction.frames.R(1);
      n = reshape(normalizeVec(double(normal)),[],1);
      r = reshape(double(origin),[],1);
      A = n';
      b = -n'*r;
      obj = obj@drakeFunction.Affine(input_frame,output_frame,A,b);
    end
  end
end
