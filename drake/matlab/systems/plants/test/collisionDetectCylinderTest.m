function collisionDetectCylinderTest(varargin)
  %NOTEST 
  % Bullet requires much more memory/many more iterations to compute
  % penetration depths accurately for cylinders than it does for the
  % other geometries we use.  Therefore, this test will fail until we
  % either increase the iterations and memory allowed for all
  % geometries, or fix Bullet to allow us to set the allowable
  % iterations and memory on a per-pair basis (right-now they're pound
  % defined).
  %
  % See https://github.com/avalenzu/bullet-pod/commit/76a04837a3acca944803737186d4d7a40330b35f
  %
  if (nargin < 2)
    tol = 2e-6;
  else
    typecheck(varargin{1},'double');
    tol = varargin{1};
    varargin(1) = [];
  end
  collisionDetectTest('Cylinder.urdf',tol,varargin{:});
end
