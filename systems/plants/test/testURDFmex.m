function testURDFmex

for urdf = {'FallingBrick.urdf',...
    '../../../examples/FurutaPendulum/FurutaPendulum.urdf', ...
    '../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'};

  urdffile = GetFullPath(urdf{1});
  fprintf(1,'testing %s\n', urdffile);
  r = RigidBodyManipulator(urdffile,struct('floating',true));
  
  q = zeros(getNumDOF(r),1);
  kinsol = doKinematics(r,q);
  
  [retval,out] = system(['./urdf_kin_test ',urdffile,sprintf(' %f',q)]);
  valuecheck(retval,0);
  out = textscan(out,'%f','delimiter',',');
  out = reshape(out{1},6,[])';
  
  for i=1:getNumBodies(r)
    fprintf(1,'%s\n',getLinkName(r,i));
    x = forwardKin(r,kinsol,i,zeros(3,1),1);
    valuecheck(out(i,:)',x);
  end
  
end

end