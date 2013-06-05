function testURDFmex

if (~exist('urdf_kin_test','file'))
  warning('testURDFmex requires that urdf_kin_test is built (from the command line).  skipping this test');
  return;
end

tol = 1e-4; % low tolerance because i'm writing finite precision strings to and from the ascii terminal

for urdf = {'./FallingBrick.urdf',...
    '../../../examples/FurutaPendulum/FurutaPendulum.urdf', ...
    '../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'};

  urdffile = GetFullPath(urdf{1});
  fprintf(1,'testing %s\n', urdffile);
  r = RigidBodyManipulator(urdffile,struct('floating',true));
  
  q = 0*rand(getNumDOF(r),1);
  kinsol = doKinematics(r,q);
  
  [retval,outstr] = system(['./urdf_kin_test ',urdffile,sprintf(' %f',q)]);
  valuecheck(retval,0);
  out = textscan(outstr,'%s %f %f %f %f %f %f');%,'delimiter',',');
  
  for i=1:getNumBodies(r)
    b = findLink(r,out{1}{i});
    pt = cellfun(@(a) a(i),out(2:end))';
    x = forwardKin(r,kinsol,b,zeros(3,1),1);
    valuecheck(pt,x,tol);  
  end
  
end

end