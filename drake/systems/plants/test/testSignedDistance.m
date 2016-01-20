function testSignedDistance

visualize = 0;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();

s = 'FallingBrick.urdf';
p = TimeSteppingRigidBodyManipulator(s,.01,options);
p = p.addRobotFromURDF(s,[],[],options);
x0 = [randn;randn;1+randn;rpy2quat(randn(3,1));
      randn;randn;1+randn;rpy2quat(randn(3,1));
      zeros(13*2, 1)];

x0 = p.resolveConstraints(x0);
N = 20000;
points = [(rand(1, N)-0.5)*1.5; (rand(1,N)-0.5)*1.5; (rand(1,N)-0.1)*2.5];

kinsol = p.doKinematics(x0(1:p.getNumPositions));

t0 = tic();
[phi, normal, x, body_idx] = p.getManipulator.signedDistances(kinsol, points, false);
toc(t0)


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
