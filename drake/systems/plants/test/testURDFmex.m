function testURDFmex(urdfs)

urdf_kin_test = fullfile(get_drake_binary_dir(), '/bin/urdfKinTest');
urdf_manipulator_dynamics_test = fullfile(get_drake_binary_dir(), '/bin/urdfManipulatorDynamicsTest');
if ispc
  urdf_kin_test = [urdf_kin_test,'.exe'];
  urdf_manipulator_dynamics_test = [urdf_manipulator_dynamics_test,'.exe'];
end

if (~exist(urdf_kin_test,'file'))
  error('Drake:MissingDependency','testURDFmex requires that urdfKinTest is built (from the command line).  skipping this test');
end
if (~exist(urdf_manipulator_dynamics_test,'file'))
  error('Drake:MissingDependency','testURDFmex requires that urdfManipulatorDynamicsTest is built (from the command line).  skipping this test');
end

tol = .1; % low tolerance because i'm writing finite precision strings to and from the ascii terminal

if nargin<1 || isempty(urdfs)
  urdfs = allURDFs();
elseif ~iscell(urdfs)
  urdfs = {urdfs};
end

for urdf = urdfs'
  urdffile = GetFullPath(urdf{1});
  fprintf(1,'testing %s\n', urdffile);
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
  try 
    r = RigidBodyManipulator(urdffile,struct('floating',true));
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
  
  q = getRandomConfiguration(r);
  kinsol = doKinematics(r,q);

  [retval,outstr] = systemWCMakeEnv([urdf_kin_test,' ',urdffile,sprintf(' %f',q),' 2> /dev/null']);
  valuecheck(retval,0);
  outstr_cell = regexp(outstr, '=======', 'split');  outstr = outstr_cell{2}(2:end);
  outstr_cell = regexp(outstr, 'phi =', 'split'); outstr = outstr_cell{1}; 
  out = textscan(outstr,'%s %f %f %f %f %f %f');%,'delimiter',',');
  
  num_q = getNumPositions(r);
  num_v = getNumVelocities(r);
  P = sparse(0,num_v);  % map from matlab coords to c++ coords
  linkid_matlab = [];
  linkid_c = [];
  for i=1:length(out{1})
    try
      linkid_matlab(end+1) = findLinkId(r,out{1}{i},-1,1);
      linkid_c(end+1)=i;
    catch ex
      if (strcmp(ex.identifier,'Drake:RigidBodyManipulator:WeldedLinkInd'))
        % skip welded cases (at least until we implement
        % https://github.com/RobotLocomotion/drake/issues/687
        disp(['skipping ',out{1}{i},' because it''s been welded']);
        continue;
      else
        rethrow(ex);
      end
    end
    if r.body(linkid_matlab(end)).parent~=0
      inds = r.body(linkid_matlab(end)).position_num;
      for j=inds'
        P(end+1,j)=1;
      end
    end
  end
  num_qc = size(P,1);  % todo: update this when num_q ~= num_v
  num_vc = size(P,1);
  num_u = length(r.actuator);  % not getNumInputs, because RigidBodyForce elements are not parsed in C++ yet
  
  if ~isequal(P,eye(size(P)))  % eventually want to get rid of P (but this still triggers for a few urdfs)
    % then rerun with remapped inputs
    [retval,outstr] = systemWCMakeEnv([urdf_kin_test,' ',urdffile,sprintf(' %f',P*q),' 2> /dev/null']);
    valuecheck(retval,0);
    outstr_cell = regexp(outstr, '=======', 'split');  outstr = outstr_cell{2}(2:end);
    out = textscan(outstr,'%s %f %f %f %f %f %f');%,'delimiter',',');
  end
  
  for i=1:length(linkid_matlab)  
    pt = cellfun(@(a) a(linkid_c(i)),out(2:end))';
    x = forwardKin(r,kinsol,linkid_matlab(i),zeros(3,1),1);

    pt(4:6) = mod(pt(4:6),2*pi);
    x(4:6) = mod(x(4:6),2*pi);
    [tf,errstr]=valuecheck(pt,x,tol);
    if ~tf,
      disp(['checking ',r.body(linkid_matlab).linkname]); 
      error('Drake:ValueCheck',errstr); 
    end
  end

  %% test manipulator dynamics
  if any(any([r.body.Iaddedmass]))
    disp(' this model has added mass (not implemented in c++ yet)');
    continue;
  end
  if length(r.force)>0
    disp(' this model had force elements (not implemented in c++ yet)'); 
    continue;
  end
  
  v = rand(getNumVelocities(r),1);

  [retval,outstr] = systemWCMakeEnv([urdf_manipulator_dynamics_test,' ',urdffile,sprintf(' %f',[P*q;P*v]),' 2> /dev/null']);
  valuecheck(retval,0);
  outstr_cell = regexp(outstr, '=======', 'split');  outstr = outstr_cell{2}(2:end);

  num_bodies = textscan(outstr,'%d',1); num_bodies=num_bodies{1};
  linknames = textscan(outstr,'%s',num_bodies,'HeaderLines',1); linknames = linknames{1};

  Hcpp = textscan(outstr,'%f','HeaderLines',1+num_bodies);
  index = num_vc^2;
  Ccpp = reshape(Hcpp{1}(index+(1:num_vc)),num_vc,1);
  index = index + num_vc;
  Bcpp = reshape(Hcpp{1}(index+(1:num_u*num_vc)),num_u,num_vc)';
  index = index + numel(Bcpp);
  num_phi = 6*length(r.loop);
  phicpp = reshape(Hcpp{1}(index+(1:num_phi)),num_phi,1);
  index = index + num_phi;
  Jcpp = reshape(Hcpp{1}(index+(1:num_phi*num_qc)),num_qc,num_phi)';
  Hcpp = reshape(Hcpp{1}(1:num_vc^2),num_vc,num_vc);

  Hcpp = P'*Hcpp*P;
  Ccpp = P'*Ccpp;
  Bcpp = P'*Bcpp;
  [H,C,B] = manipulatorDynamics(r,q,v);
  B = B(:,1:num_u); % ignore rigid body force inputs

  has_friction = ([r.body.damping]~=0) | ([r.body.coulomb_friction]~=0);
  has_friction = [r.body(has_friction).velocity_num];
  no_friction = true(r.getNumVelocities(),1); no_friction(has_friction)=false;
  if ~isempty(has_friction)
    disp(' note: friction indices will be ignored (not implemented in c++ yet)');
  end
  
  % low tolerance because i'm just parsing the ascii printouts
  valuecheck(Hcpp,H,tol);
  valuecheck(Ccpp(no_friction),C(no_friction),tol);
  valuecheck(Bcpp,B,tol);
  if num_phi>0
    [phi,J] = positionConstraints(r,q);
    Jcpp = Jcpp*P;
    valuecheck(phicpp,phi,tol/10);
    valuecheck(Jcpp,J,tol);
  end
end

end
