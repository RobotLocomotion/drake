function AllBodiesClosestDistanceConstraintTest(n,draw_pause)
  if nargin < 2
    draw_pause = [];
  end
  if nargin < 1
    n = [];
  end
  options.floating = true;
  w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  r = RigidBodyManipulator([getDrakePath(), '/examples/Atlas/urdf/atlas_convex_hull.urdf'],options);
  warning(w);

  % Alternatively, one could remove the collision elements for these links from
  % the urdf ...
  ignored_bodies = {'ltorso','mtorso','r_talus','l_talus'};
  r = addLinksToCollisionFilterGroup(r,ignored_bodies,'no_collision',1);
  r = compile(r);

  S = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
  q_nom = S.xstar(1:r.getNumPositions());
  options = optimset('TolFun',1e-5,'TolX',1e-6);
  constraintTester('AllBodiesClosestDistanceConstraintTest',r,@makeCon, @(r) q_nom, @makeQseed, n, draw_pause,options,@objFun);

  function [f,Gf] = objFun(q,q_nom,q_seed)
    f = (q-q_seed)'*(q-q_seed);
    Gf = 2*(q-q_seed);
  end

  function q_seed = makeQseed(r)
    [joint_limit_min,joint_limit_max] = r.getJointLimits();
    joint_range = (joint_limit_max - joint_limit_min);
    q_seed = q_nom;
    q_seed(7:end) = q_seed(7:end) + (rand(size(q_seed(7:end))) - 0.5).*joint_range(7:end)/2;
  end

  function con = makeCon(r)
    con = AllBodiesClosestDistanceConstraint(r,0.02,1e3,[0,1]);
  end
end
