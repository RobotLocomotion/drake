function testURDFmex

urdf_kin_test = '../../../pod-build/bin/urdfKinTest';
urdf_manipulator_dynamics_test = '../../../pod-build/bin/urdfManipulatorDynamicsTest';
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

tol = 1e-2; % low tolerance because i'm writing finite precision strings to and from the ascii terminal

for urdf = allURDFs()'
  urdffile = GetFullPath(urdf{1});
  fprintf(1,'testing %s\n', urdffile);
  r = RigidBodyManipulator(urdffile,struct('floating',true,'use_new_kinsol',true));

  if ~isempty(r.param_db)
    disp(' this model has parameters (not implemented in c++ yet), so will be skipped');
    continue;
  end
  q = 0*rand(getNumPositions(r),1);
  kinsol = doKinematics(r,q);

  [retval,outstr] = systemWCMakeEnv([urdf_kin_test,' ',urdffile,sprintf(' %f',q),' 2> /dev/null']);
  valuecheck(retval,0);
  out = textscan(outstr,'%s %f %f %f %f %f %f');%,'delimiter',',');
  
  for i=1:getNumBodies(r)
    try
      b = findLinkId(r,out{1}{i},-1,1);
    catch ex
      % skip welded cases (at least until we implement
      % https://github.com/RobotLocomotion/drake/issues/687
      disp(['skipping ',out{1}{i},' because it''s been welded']);
      continue;
    end
    pt = cellfun(@(a) a(i),out(2:end))';
    x = forwardKin(r,kinsol,b,zeros(3,1),1);

    pt(4:6) = mod(pt(4:6),2*pi);
    x(4:6) = mod(x(4:6),2*pi);
    valuecheck(pt,x,tol);
  end

  %% test manipulator dynamics
  v = 0*rand(getNumVelocities(r),1);

  [retval,outstr] = systemWCMakeEnv([urdf_manipulator_dynamics_test,' ',urdffile,sprintf(' %f',q),' 2> /dev/null']);
  valuecheck(retval,0);

  num_bodies = textscan(outstr,'%d',1); num_bodies=num_bodies{1};
  linknames = textscan(outstr,'%s',num_bodies,'HeaderLines',1); linknames = linknames{1};

  num_q = getNumPositions(r);
  num_v = getNumVelocities(r);
  P = sparse(0,num_v);
  for i=1:num_bodies
    try
      bi = findLinkId(r,linknames{i},-1,1);
    catch
      % welded case
      P(end+1,1)=0;
      continue;
    end
    if r.body(bi).parent~=0
      inds = r.body(bi).position_num;
      for j=inds'
        P(end+1,j)=1;
      end
    end
  end
  num_qc = size(P,1);  % todo: update this when num_q ~= num_v
  num_vc = size(P,1);

  Hcpp = textscan(outstr,'%f','HeaderLines',1+num_bodies);
  index = num_vc^2;
  Ccpp = reshape(Hcpp{1}(index+(1:num_vc)),num_vc,1);
  index = index + num_vc;
  Bcpp = reshape(Hcpp{1}(index+(1:getNumInputs(r)*num_vc)),getNumInputs(r),num_vc)';
  index = index + numel(Bcpp);
  num_phi = 3*length(r.loop);
  phicpp = reshape(Hcpp{1}(index+(1:num_phi)),num_phi,1);
  index = index + num_phi;
  Jcpp = reshape(Hcpp{1}(index+(1:num_phi*num_qc)),num_qc,num_phi)';
  Hcpp = reshape(Hcpp{1}(1:num_vc^2),num_vc,num_vc);

  Hcpp = P'*Hcpp*P;
  Ccpp = P'*Ccpp;
  Bcpp = P'*Bcpp;
  [H,C,B] = manipulatorDynamics(r,q,v);

  % low tolerance because i'm just parsing the ascii printouts
  valuecheck(Hcpp,H,tol);
  valuecheck(Ccpp,C,tol);
  valuecheck(Bcpp,B,tol);
  if num_phi>0
    [phi,J] = positionConstraints(r,q);
    Jcpp = Jcpp*P;
    valuecheck(phicpp,phi,tol);
    valuecheck(Jcpp,J,tol);
  end
end

end
