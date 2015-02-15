function testURDFmex

urdf_kin_test = '../../../pod-build/bin/urdf_kin_test';
urdf_manipulator_dynamics_test = '../../../pod-build/bin/urdf_manipulator_dynamics_test';
if ispc
  urdf_kin_test = [urdf_kin_test,'.exe'];
  urdf_manipulator_dynamics_test = [urdf_manipulator_dynamics_test,'.exe'];
end

if (~exist(urdf_kin_test,'file'))
  error('Drake:MissingDependency','testURDFmex requires that urdf_kin_test is built (from the command line).  skipping this test');
end

tol = 1e-4; % low tolerance because i'm writing finite precision strings to and from the ascii terminal

for urdf = {'./FallingBrick.urdf',...
    '../../../examples/FurutaPendulum/FurutaPendulum.urdf', ...
    '../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'};

  urdffile = GetFullPath(urdf{1});
  fprintf(1,'testing %s\n', urdffile);
  r = RigidBodyManipulator(urdffile,struct('floating',true));
    
  q = 0*rand(getNumPositions(r),1);
  kinsol = doKinematics(r,q);
  
  [retval,outstr] = systemWCMakeEnv([urdf_kin_test,' ',urdffile,sprintf(' %f',q),' 2> /dev/null']);
  valuecheck(retval,0);
  out = textscan(outstr,'%s %f %f %f %f %f %f');%,'delimiter',',');
  
  for i=1:getNumBodies(r)
    b = findLinkId(r,out{1}{i});
    pt = cellfun(@(a) a(i),out(2:end))';
    x = forwardKin(r,kinsol,b,zeros(3,1),1);
    valuecheck(pt,x,tol);  
  end
  
  %% test manipulator dynamics
  v = 0*rand(getNumVelocities(r),1);
  
  [retval,outstr] = systemWCMakeEnv([urdf_manipulator_dynamics_test,' ',urdffile,sprintf(' %f',q),' 2> /dev/null']);
  valuecheck(retval,0);
  
  num_bodies = textscan(outstr,'%d',1); num_bodies=num_bodies{1};
  linknames = textscan(outstr,'%s',num_bodies,'HeaderLines',1); linknames = linknames{1};

  num_v = getNumVelocities(r);
  map = [];
  for i=1:num_bodies
    inds = r.body(r.findLinkId(linknames{i})).position_num;
    if any(inds>0)
      map = [map;inds];
    end
  end
  P = sparse(1:num_v,map,ones(num_v,1),num_v,num_v);
  
  Hcpp = textscan(outstr,'%f','HeaderLines',1+num_bodies);
  Ccpp = reshape(Hcpp{1}(num_v^2+(1:num_v)),[],1);
  Hcpp = reshape(Hcpp{1}(1:num_v^2),getNumVelocities(r),getNumVelocities(r));
  Hcpp = P'*Hcpp*P;
  
  Ccpp = P'*Ccpp;
  
  [H,C] = manipulatorDynamics(r,q,v);
  
  % low tolerance because i'm just parsing the ascii printouts
  valuecheck(Hcpp,H,1e-2); 
  valuecheck(Ccpp,C,1e-2);  
  
end

end
