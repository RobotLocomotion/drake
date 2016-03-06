function  robot = planarN( n )

% planarN  create an n-link planar robot.
% planarN(n) creates an all-revolute n-link planar robot with identical links.

robot.NB = n;
robot.parent = [0:n-1];
robot.pitch = zeros(1,n);

for i = 1:n
  if i == 1
    robot.Xtree{i} = Xtrans([0 0 0]);
  else
    robot.Xtree{i} = Xtrans([1 0 0]);
  end
  robot.I{i} = mcI( 1, [0.5 0 0], diag([0.01,1/12,1/12]) );
end

robot.appearance{1} = {...
  { 'box', [-0.2, 0.2; -0.3, 0.3; -0.2, -0.08] } };

for i = 1:n
  robot.appearance{i+1} = {...
    { 'box', [0 1; -0.07 0.07; -0.05 0.05] },...
    { 'cyl', [0 0 0], 0.1, 0.16, 'Z' } };
end
