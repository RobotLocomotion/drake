function bullet_raycast_test
% tests raycasting in bullet

checkDependency('bullet');

use_visualizer = 0;

if (use_visualizer == 1)
    checkDependency('lcmgl');
end

% set up a world with two poles and a brick in it

options.floating = true;
options.terrain = [];

rb = RigidBodyManipulator([getDrakePath '/systems/plants/test/FallingBrick.urdf'], options);


% world is rb.body(1)

world = rb.getBody(1);


% add obstacles to the world

obstacle1 = RigidBodyCapsule(1, 10);
obstacle1.T(1:3, 4) = [ 10; 0; 0 ];

world.collision_geometry{end+1} = obstacle1;
world.visual_geometry{end+1} = obstacle1;

obstacle2 = RigidBodyCapsule(1, 10);
obstacle2.T(1:3, 4) = [ -10; 0; 0 ];

world.collision_geometry{end+1} = obstacle2;
world.visual_geometry{end+1} = obstacle2;

% put the new world back into the rb
rb = rb.setBody(1, world);

% we're about to throw a warning, so supress it but immediately restore it
warn_state = warning('query', 'Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints');

rb = compile(rb);

% reset warning state
warning(warn_state.state, 'Drake:RigidBodyManipulator:UnsupportedContactPoints');

% test raycasting

zheight = 0;

q0 = rand(rb.getNumPositions(), 1);
q0(2) = 10;
q0(3) = 10; % the block shouldn't be in the sensor's view for a 2D sweep
x0 = [q0; rand(rb.getNumVelocities(), 1)]; % full state (positions, velocities)

if (use_visualizer == 1)
    v = rb.constructVisualizer();
    v.draw(0, x0);
end

kinsol = doKinematics(rb, q0);

distance = [];
angles = [];

% test the non vectorized case
for angle = 0:.01:2*pi
    
    origin = [0; 0; zheight];
    
    point_on_ray = [cos(angle); sin(angle); 0] * 1000;
    point_on_ray(3) = zheight;
    
    angles(end+1) = angle;
    distance(end+1) = collisionRaycast(rb, kinsol, origin, point_on_ray);
    
end


% test the vectorized case
angles2 = 0:0.01:2*pi;
origins = repmat(origin, 1, length(angles2));
endpoints = [cos(angles2); sin(angles2); 0*angles2] * 1000;
endpoints(3,:) = zheight;


distance2 = collisionRaycast(rb, kinsol, origins, endpoints);


if (distance' ~= distance2)
  error('Error: vecotrized and non-vectorized raycast mismatch.');
end

if (use_visualizer == 1)
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'raycasting');
    lcmgl.glColor3f(1, 0, 0);
end

% count the number of hits we get
hit_num = 0;

for i=1:length(distance)
    if (distance(i) > 0)
        % got a hit
        hit_num = hit_num + 1;
        
        point = [cos(angles(i)); sin(angles(i)); 0] * distance(i);
        point(3) = zheight;
        
        % draw point hit and lines
        if (use_visualizer == 1)
            lcmgl.sphere(point, .1, 20, 20);
            lcmgl.line3(origin(1), origin(2), origin(3), point(1), point(2), point(3));
        end
        
        % make sure the hit is on one of the poles
        
        % poles are located at (-10, 0, 0) and (10, 0, 0)
        % with radius 1
        
        if (point(1) > 0)
            if (point(1) >= 8.5 && point(1) <= 11.5 && point (2) >= -1 && point(2) <= 1)
                % ok
            else
                error('Raycasting point did not end on a cylinder.');
            end
        else
            % on the other pole
            if (point(1) <= -8.5 && point(1) >= -11.5 && point (2) >= -1 && point(2) <= 1)
                % ok
            else
                error('Raycasting point did not end on a cylinder.');
            end
        end
            
    end
end

if (use_visualizer == 1)
    lcmgl.switchBuffers;
end

valuecheck(hit_num, 41); % we should get 41 hits on the poles


