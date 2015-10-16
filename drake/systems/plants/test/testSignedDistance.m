function testSignedDistance

options.floating = true;
options.terrain = RigidBodyFlatTerrain();

% p = Atlas('/home/gizatt/drc/software/drake/drake/examples/Atlas/urdf/atlas_convex_hull.urdf',options);
% load('/home/gizatt/drc/software/drake/drake/examples/Atlas/data/atlas_fp.mat');
% x0 = xstar;
% N = 20000;
% points = [(rand(1, N)-0.5)*1.5; (rand(1,N)-0.5)*1.5; (rand(1,N)-0.1)*2.5];


p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);
x0 = [rand(6,1)+[0;0;1;0;0;0]; 0; 0; 0; zeros(6,1)];
N = 20000;
points = [(rand(1, N)-0.5)*4; (rand(1,N)-0.5)*4; (rand(1,N)-0.1)*4];

% 
% lcmgl = LCMGLClient('testSDF_debug');
% lcmgl.glColor3f(1,0,0);
% lcmgl.points(points(1,:), points(2,:), points(3,:));
% lcmgl.switchBuffers();


v = p.constructVisualizer();
v.drawWrapper(0,x0);

kinsol = p.doKinematics(x0(1:p.getNumPositions));

t0 = tic();
[phi, normal, x, body_idx] = p.getManipulator.signedDistances(kinsol, points, false);
toc(t0)

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