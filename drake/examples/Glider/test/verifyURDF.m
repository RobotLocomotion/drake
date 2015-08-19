function verifyURDF()
% Tests the URDF implementation of the glider model versus the GliderPlant
% that was adapted from Rick Cory's original model.  The wing areas may not
% exactly match the physical glider used--they were initially picked with
% a known working system so the dynamics would match

  % state:  
  %  x(1) - x position
  %  x(2) - z position
  %  x(3) - pitch (theta)
  %  x(4) - elevator (phi)
  %  x(5) - x velocity
  %  x(6) - z velocity
  %  x(7) - pitch velocity (thetadot)
  % input:
  %  u(1) - elevator velocity (phidot)
cd ..
%disp('constructing a Planar glider')
options.floating = true;
%p = TimeSteppingRigidBodyManipulator('Glider.URDF',.001, options);
p = PlanarRigidBodyManipulator('GliderBalanced.urdf', options);
%    [X Z Pitch El Vx Vz PitDot Velev]
for i = 1:100
    u0 = rand(1)-.5;
    pitch = rand(1)-.5;
    phi = rand(1)-.5;
    xvel = rand(1)*3+4;
    zvel = rand(1)*2-1;
    pitchdot = rand(1)-.5;  
    xp = [0 0   -pitch   -phi  xvel  zvel    -pitchdot    -u0]';
    % URDF's are defined with positive pitch = down (y-axis rotation)
    %GliderPlant uses positive pitch = up.
    xgp = [0 0   pitch   phi  xvel  zvel    pitchdot]';
    gp = GliderPlant();
    glider_xdot = gp.dynamics(0,xgp, u0);
    %Wrapper because the joint is velocity-controlled.
    %u_needed = -.013
    u_needed = computeAccel(p,xp,-u0);
    urdf_xdot = p.dynamics(0,xp,u_needed);
    %because pitch axes are reversed between models--also for elevator(input)
    urdf_xdot(3) = -urdf_xdot(3);
    urdf_xdot(4) = -urdf_xdot(4);
    urdf_xdot(7) = -urdf_xdot(7);
    %Don't use valuecheck because for larger accelerations (~10), they are
    %accurate to ~2%, but small accelerations (~.01) may be off by ~5%, but
    %only .0005 absolute value, both of which are acceptable --Tim
    err = abs(glider_xdot-urdf_xdot(1:7));
    if any((err(1:7)>.05).*(err(1:7)./abs(glider_xdot(1:7))>.03))
        error('Values don''t match.  Expected \n%s\n but got \n%s', mat2str(glider_xdot), mat2str(urdf_xdot(1:7)));
    end
    %      error('Dynamics of GliderPlant and GliderBalanced.URDF do not match.  Ensure the elevator is almost massless and inertia-less, else you broke something.');
end
cd test
    function u = computeAccel(p, x, vel)
        qdd = 0;%10*(x(8)-vel);
        [H,C,B] = manipulatorDynamics(p,x(1:4),x(5:8));
        H11 = H(1:3,1:3);
        H12 = H(1:3,4);
        H21 = H(4,1:3);
        H22 = H(4,4);
        C1 = C(1:3);
        C2 = C(4);
        u = H21*(inv(H11)*-(H12*qdd+C1)) + H22*qdd + C2;
    end
end
