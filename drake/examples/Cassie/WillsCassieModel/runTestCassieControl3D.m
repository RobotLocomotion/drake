function runTestCassieControl3D

% values for harmonic motion of legs
l_ret = -.15;
l0 = 0;
w = 2*pi/.7;

planar = false;
dt = .004;
sim_duration = 10.0;

% push_vel = initial starting velocity in x direction, push_vel2 = initial starting velocity in y directio
push_vel =  0.0;
push_vel2 = 0.0;

% setup options
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.ignore_self_collisions = true;
options.use_bullet = false;

% choose 2d or 3d. Default 3d, if you want 2d you'll need to change the
% controller input dimensions
if planar
    r = TimeSteppingRigidBodyManipulator(PlanarRigidBodyManipulator('3DURDF_Centroid.urdf', options), dt, options);
else
    r = TimeSteppingRigidBodyManipulator(RigidBodyManipulator('3DURDF_Centroid.urdf', options), dt, options);
end

v = r.constructVisualizer();

%v.inspector;

% call controller
c = testCassieControl3D(r);
sys = feedback(r,c);

if planar
    x0 = [0;.91;0;zeros(8,1);push_vel;0;0;0;0;0;0;0;0];
else
    x0 = [0;0;.91;0;0;0;zeros(8,1);push_vel;push_vel2;0;zeros(11,1)];
end

traj = simulate(sys,[0 sim_duration],x0);

v.playback_speed = .5;
playback(v,traj,struct('slider',true));



% GET GRAPHS

% handy dictionary
s = containers.Map({'base_x',
                              'base_y',
                              'base_z',
                              'base_roll',
                              'base_pitch',
                              'base_yaw',
                              'shoulder_left',
                              'shoulder_left2',
                              'shoulder_right',
                              'shoulder_right2',
                              'ankle_left',
                              'ankle_right',
                              'left_foot_to_toe',
                              'right_foot_to_toe',
                              'base_xdot',
                              'base_ydot',
                              'base_zdot',
                              'base_rolldot',
                              'base_pitchdot',
                              'base_yawdot',
                              'shoulder_leftdot',
                              'shoulder_left2dot',
                              'shoulder_rightdot',
                              'shoulder_right2dot',
                              'ankle_leftdot',
                              'ankle_rightdot',
                              'left_foot_to_toedot',
                              'right_foot_to_toedot'}, [1:28]);
 
                          
% TO GET GRAPH, uncomment
%{ 
                          
% ideal function of legs
fun_legs = @(t) [((sin(w*t) < 0) * (l0 + l_ret * sin(w*t)) + (sin(w*t) >= 0) * (l0)), ((sin(w*t) > 0) * (l0 - l_ret * sin(w*t)) + (sin(w*t) <= 0) * (l0))];
ftraj = FunctionHandleTrajectory([fun_legs],2,[0 sim_duration]);


% get points in trajectory
ts = traj.getBreaks();
xs = traj.eval(ts);
fs = eval(ftraj,ts);


qs = xs(1:14, :);

w = @(x) r.contactConstraints(r.doKinematics(x));

[rows, columns] = size(qs);
for col = 1 : columns
    thisColumn = qs(:, col);
    e = w(thisColumn);
    constraintDistance(:,col) = e;
end



figure(1); 
subplot(3,1,1);
plot(ts,fs(1,:),ts,xs(s('ankle_left'),:));
subplot(3,1,2);
plot(ts,fs(2,:),ts,xs(s('ankle_right'),:));
subplot(3,1,3);
plot(ts,xs(s('ankle_left'),:),ts,xs(s('ankle_right'),:));
%plot(ts,constraintDistance(3,:),ts,constraintDistance(4,:));


figure(2);
subplot(3,1,1);
plot(ts, xs(s('base_xdot'),:))
subplot(3,1,2);
plot(ts, xs(s('shoulder_left2'),:))
subplot(3,1,3);
plot(ts, xs(s('shoulder_right2'),:))


figure(3);
subplot(3,1,1);
plot(ts, xs(s('base_ydot'),:))
subplot(3,1,2);
plot(ts, xs(s('shoulder_left'),:))
subplot(3,1,3);
plot(ts, xs(s('shoulder_right'),:))

figure(4);
plot(ts, xs((s('base_ydot')^2 + s('base_ydot')^2)^.5,:))


%assignin('base', 'traj', traj);

%}




