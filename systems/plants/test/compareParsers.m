function compareParsers(urdfs)

if nargin<1 || isempty(urdfs)
  urdfs = allURDFS();
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
    r = RigidBodyManipulator(urdffile,struct('floating',true,'use_new_kinsol',true));
  catch ex
    if strncmp(ex.identifier,'Drake:MissingDependency:',24)
      disp(' skipping due to a missing dependency'); 
      continue;
    end
  end
  
  if ~isempty(r.param_db)
    disp(' this model has parameters (not implemented in c++ yet), so will be skipped');
    continue;
  end
  
  compareParsersmex(r.mex_model_ptr,urdfs{1});
end