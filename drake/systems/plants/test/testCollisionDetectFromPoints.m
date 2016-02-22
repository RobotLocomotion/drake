function testCollisionDetectFromPoints
checkDependency('bullet');

visualize = 0;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();

p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);
p = p.addRobotFromURDF( 'FallingBrickContactPoints.urdf',[],[],options);
x0 = [0.0; 0.5; 1; 0;0;0;
      0.0; -0.5; 1; 0;0;0;
      zeros(12*2, 1)];

x0 = p.resolveConstraints(x0);

%test it with just a few points that we know the answers for
points = [0, 0.25, 1; % surface of the brick with proper collision geometry
          0, 0, 1; % between bricks, 0.25 from each surface
          0, -0.25, 1; % 0.5 from brick with proper collision geometry, and far from corners of the other brick
          1,  -0.25, 1.75; %0.25 above a corner of the brick with zerorad points at corners
          ].';
kinsol = p.doKinematics(x0(1:p.getNumPositions));
[phi, normal, x, body_x, body_idx] = p.getManipulator.collisionDetectFromPoints(kinsol, points, false);

valuecheck(phi, [0; 0.25; .5; .25], 1E-6);
valuecheck(body_idx, [2; 2; 2; 3;], 1E-6);
valuecheck(x, [0, 0.25, 1;
                0, 0.25, 1;
                0, 0.25, 1;
                1, -0.25, 1.5].', 1E-6);
valuecheck(normal, [0, -1, 0;
                    0, -1, 0;
                    0, -1, 0;
                    0, 0, 1].', 1E-6);
                
                
                
% test it with a ton of points
N = 20000;
points = [(rand(1, N)-0.5)*1.5; (rand(1,N)-0.5)*1.5; (rand(1,N)-0.1)*2.5];

t0 = tic();
[phi, normal, x, body_x, body_idx] = p.getManipulator.collisionDetectFromPoints(kinsol, points, false);
toc(t0)

% pretty rendering
if (visualize)
    v = p.constructVisualizer();
    v.drawWrapper(0,x0);
    PHI = 0;
    BODY = 1;
    color_by = PHI;
    lcmgl = LCMGLClient('testSDF_debug');
    if (color_by == PHI)
        mindist = -0.5;
        maxdist = 0.5;
        phi(phi < mindist) = mindist;
        phi(phi > maxdist) = maxdist;
        norm_phi = (phi - mindist) / (maxdist - mindist);
        for i = 1:size(points,2)
            lcmgl.glColor3f(1-norm_phi(i)^2,norm_phi(i)^2,0);
            lcmgl.points(points(1,i), points(2,i), points(3,i));
        end
    elseif (color_by == BODY)
        for i = 1:size(points,2)
            lcmgl.glColor3f(mod(body_idx(i),2), 1-mod(body_idx(i),2),1 );
            lcmgl.points(points(1,i), points(2,i), points(3,i));
        end
    end
    lcmgl.switchBuffers();
end
