function runTrajOptRRT

%addpath(fullfile(pwd,'..'));
%p = PendulumPlant();

p = PlanarRigidBodyManipulator('../Pendulum.urdf');
findTrajectory(p,[0;0],[pi;0],@randSample,struct());


end

function xs = randSample()

xlim = [-pi/2,3*pi/2; -10,10];
xs = (xlim(:,2)-xlim(:,1)).*rand(2,1) +  xlim(:,1);

end