function compareParsers(urdfs)

if nargin<1 || isempty(urdfs)
  % The following line was commented out because it does not work on
  % Ubuntu or OSX. See: https://github.com/RobotLocomotion/drake/issues/2194
  % urdfs = allURDFs();

  urdfs = {'ActuatedPendulum.urdf';
           'ball.urdf';
           'block_offset.urdf';
           'brick1.urdf';
           'brick2.urdf';
           'brick3.urdf';
           'brick4.urdf';
           'brick_point_contact.urdf';
           'Capsule.urdf';
           'Cylinder.urdf';
           'DoublePendWBiceptSpring.urdf';
           'FallingBrickBetterCollisionGeometry.urdf';
           'FallingBrickContactPoints.urdf';
           'FallingBrick.urdf';
           'fixedJointTest.urdf';
           'FloatingMassSpringDamper.urdf';
           'ground_plane.urdf';
           'MassSpringDamperThrust.urdf';
           'MassSpringDamper.urdf';
           'PointMass.urdf';
           'ShiftedPointMass.urdf';
           'snake.urdf';
           'SpringPendulum.urdf';
           'testRigidBodyBluffBody.urdf';
           'testRigidBodyPropellor.urdf';
           'testRigidBodyWingChangingParams.urdf';
           'testRigidBodyWingWithControlSurfaceOnlyControlSurface.urdf';
           'testRigidBodyWingWithControlSurface.urdf';
           'testThrust.urdf';
           'TestWing.urdf';
           'TorsionalSpring.urdf';
           'valve_task_wall.urdf'}

elseif ~iscell(urdfs)
  urdfs = {urdfs};
end

for urdf=urdfs'
  urdffile = GetFullPath(urdf{1});

  fprintf(1,'testing %s\n', urdffile);
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
  try 
    r = RigidBodyManipulator(urdffile,struct('floating','rpy'));
  catch ex
    if strncmp(ex.identifier,'Drake:MissingDependency:',24)
      disp(' skipping due to a missing dependency'); 
      continue;
    end
    rethrow(ex);
  end
  
  if ~isempty(r.param_db)
    disp(' this model has parameters (not implemented in c++ yet), so will be skipped');
    continue;
  end
  if any(any([r.body.Iaddedmass]))
    disp(' this model has added mass (not implemented in c++ yet)');
    continue;
  end
  if length(r.force)>0
    disp(' this model had force elements (not implemented in c++ yet)'); 
    continue;
  end
  if r.getNumPositions == 0
    disp(' this model is completely static (zero degrees of freedom), so will be skipped');
    continue;
  end
  
  compareParsersmex(r.mex_model_ptr,urdffile,'rpy');
end
