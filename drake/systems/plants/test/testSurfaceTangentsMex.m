function testSurfaceTangentsMex

checkDependency('bullet');

%build an atlas model 
robot = createAtlas('rpy');
if robot.mex_model_ptr == 0
  disp('testSurfaceTangentsMex: no mex model pointer... nothing to test');
  return;
end

%random initial pose
q = getRandomConfiguration(robot);
kinsol = robot.doKinematics(q);

%get collision normals
[~,normal] = robot.collisionDetect(kinsol);

%get the results from the mexed version

d_mex  = surfaceTangentsmex(robot.mex_model_ptr, normal);

%get the results from the matlab version
temp = robot.mex_model_ptr;
robot.mex_model_ptr = 0;
d = robot.surfaceTangents(normal);
robot.mex_model_ptr = temp;

%compare d
for i = 1:size(d,2)
    valuecheck(d_mex{i}, d{i});
end

end

