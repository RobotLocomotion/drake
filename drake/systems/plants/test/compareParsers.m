function compareParsers(urdfs)

if nargin<1 || isempty(urdfs)
  urdfs = allURDFs();
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
  
  compareParsersmex(r.mex_model_ptr,urdffile,'rpy');
end
