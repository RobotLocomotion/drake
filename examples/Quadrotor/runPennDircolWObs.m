function [utraj,xtraj,prog,r] = runPennDircolWObs

% simple planning demo which takes the quadrotor from hover at x=0m to a new hover at
% x=2m with minimal thrust.


%plant = 'russ';
plant = 'penn';

if plant == 'russ'
  r = Quadrotor();
  v = constructVisualizer(r);
  disp('using russ rotor!')
elseif plant == 'penn'
  r_temp = Quadrotor();
  
  %terrain = RigidBodyFlatTerrain();
  %terrain = terrain.setGeometryColor([0, 170, 255]'/256);
  %r_temp = r_temp.setTerrain(terrain);
  %oquad = Quadrotor();
  %oquad = addRobotFromURDF(oquad, 'office.urdf');
  %v = constructVisualizer(oquad);
  r_temp = addOcean(r_temp, [.8,.45,1.25], [.20;2.5], pi/4);
  %r_temp = addTree(r_temp, [.5,.35,1.65], [-.25;5], -pi/6);
  %r_temp = addTree(r_temp, [.55,.65,1.5], [.25;7.5], pi/4);
  %r_temp = addTree(r_temp, [.55,.85,1.6], [-1.35;8.5], pi/3.7);
  %r_temp = addTree(r_temp, [.85,.95,1.65], [-1.85;5.2], -pi/3.7);
  %r_temp = addTree(r_temp, [.75,.9,1.75], [2;4.4], -pi/5);
  % Random trees to make forest bigger and more dense
  %r_temp = addTrees(r_temp, 25);
  
  v = constructVisualizer(r_temp);
  
  r = QuadWindPlant(); % Quadrotor constructor
  
  % coordinate transform from r.getOutputFrame to v.getInputFrame
  %ct = CoordinateTransform(
  
  % uncomment this to get plotting back
  %r.setOutputFrame(r_temp.getStateFrame());
  
  %v = v.setInputFrame(r.getOutputFrame());
  disp('using quad plant in wind based on penn plant!!')
end


N = 21;
minimum_duration = .1;
maximum_duration = 3;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros

if plant == 'russ'
  x0.base_z = .5; % lift the quad off the ground
elseif plant == 'penn'
  x0.z = .5; % lift the quad off the ground
end

v.draw(0,double(x0));

prog = addPlanVisualizer(r,prog);

u0 = double(nominalThrust(r));

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1); % DirectTrajectoryOptimization method

bd = inf;

seasurface = Point(getStateFrame(r));
seasurface.x = -bd;
seasurface.y = -bd;
seasurface.z = 0.2;
seasurface.roll = -bd;
seasurface.pitch = -bd;
seasurface.yaw = -bd;
seasurface.xdot = -bd;
seasurface.ydot = -bd;
seasurface.zdot = -bd;
seasurface.rolldot = -bd;
seasurface.pitchdot = -bd;
seasurface.yawdot = -bd;
seasurface.mytime = -bd;

world = Point(getStateFrame(r));
world.x = bd;
world.y = bd;
world.z = bd;
world.roll = bd;
world.pitch = bd;
world.yaw = bd;
world.xdot = bd;
world.ydot = bd;
world.zdot = bd;
world.rolldot = bd;
world.pitchdot = bd;
world.yawdot = bd;
world.mytime = bd;

Ncell = {};
for i = 1:N
  Ncell = [Ncell i];
end
prog = prog.addStateConstraint(BoundingBoxConstraint(double(seasurface),double(world)),Ncell);

%field = ObstacleField();
%field = field.GenerateRandomObstacles();
%prog = field.AddConstraints(prog);

prog = prog.addInputConstraint(ConstantConstraint(u0),1); % DirectTrajectoryOptimization method

xf = x0;                       % final conditions: translated in x
upperxf = x0;
lowerxf = x0;

if plant == 'russ'
  xf.base_x = 2;                 % translate x by 2
elseif plant == 'penn'
  upperxf.x = 6;                 % translate x
  upperxf.z = 4;                 % translate z
  upperxf.y = 0;                 % translate x
  upperxf.mytime = maximum_duration;
  
  lowerxf.x = 6;                 % translate x
  lowerxf.z = 4;                 % translate z
  lowerxf.y = 0;                 % translate x
  lowerxf.mytime = minimum_duration;
  
end



prog = prog.addStateConstraint(BoundingBoxConstraint(double(lowerxf),double(upperxf)),N);
% Constant Constraint is simpler for a time-invarying problem
% prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N); % DirectTrajectoryOptimization method
prog = prog.addInputConstraint(ConstantConstraint(u0),N); % DirectTrajectoryOptimization method

prog = prog.addRunningCost(@cost); % note: DirCol not Direct!  DircolTrajectoryOptimization method
prog = prog.addFinalCost(@finalCost); % DirectTrajectoryOptimization method

tf0 = 2;                      % initial guess at duration
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)])); % traj.init.x is a PPTrajectory < Trajectory < DrakeSystem
traj_init.u = ConstantTrajectory(u0); % traj_init.u is a ConstantTrajectory < Trajectory < DrakeSystem

info=0;
while (info~=1)
  tic
  snseti ('Verify level', 3);
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init); % DirectTrajectoryOptimization method
  toc
end










% ROBIN's TVLQR CODE

tic;
x0 = xtraj.eval(0);
tf = utraj.tspan(2);
% Q = 10*eye(13);
Q = 10 * [eye(3) zeros(3,10) ; zeros(10,3) zeros(10)];
R = eye(4);
Qf = 10*eye(13);
disp('Computing stabilizing controller with TVLQR...');
ltvsys = tvlqr(r,xtraj,utraj,Q,R,Qf);

%r2 = QuadWindPlant();
%ltvsys = ltvsys.setInputFrame(r2.getOutputFrame);
%r2 = r2.setOutputFrame(r.getStateFrame());

r2 = QuadWindPlant_Constant();
r2 = r2.setOutputFrame(r.getOutputFrame);

r2 = r2.setInputFrame(r.getInputFrame);

% NOTE: Back to feedback on original plant for now
sys = feedback(r,ltvsys);

toc;
disp('done!');

% Simulate on a different wind system
%r2 = QuadWindPlant_Constant();
%sys = feedback(r2,c);

% Cascade
%tic;
%sys = cascade(utraj, r);
%toc;

% Simulate the result
tic;
disp('Simulating the system...');
%xtraj_sim = simulate(sys,[0 tf],x0);
[xtraj_sim, out2, out3]  = simulate(sys,[0 tf],x0);
toc;
disp('done!');


% Draw the TVLQR result
xtraj_sim = xtraj_sim(1:12);
xtraj_sim = xtraj_sim.setOutputFrame(r_temp.getStateFrame());
v.playback(xtraj_sim, struct('slider', true));


% % Draw the original result
% if (nargout<1)
%    xtraj = xtraj(1:12);
%    xtraj = xtraj.setOutputFrame(r_temp.getStateFrame());
%    v2.playback(xtraj,struct('slider',true));
% end





% BEN'S CASCADE CODE

%utraj = setOutputFrame(utraj,getInputFrame(r_temp));
%sys = cascade(utraj,r);
%systraj = sys.simulate([0 utraj.tspan(2)],xtraj.eval(0));
%v.playback(systraj,struct('slider',true));






% PLOT WIND
%[winddontcare,dquadinwinddontcare] = quadwind(r,[0,0,0],0,1);

%xquad = quadpos(1);
%yquad = quadpos(2);
%zquad = quadpos(3);

%windfield = 'zero';
windfield = 'constant';
%windfield = 'linear';
%windfield = 'quadratic';
%windfield = 'sqrt';
%windfield = 'exp';
%windfield = 'difftailhead';
%windfield = 'thermals';
%windfield = 'tvsin';
%windfield = 'tlinear';




lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'Windy');

lcmgl.glColor3f(0,1,0);
if strcmp(windfield, 'difftailhead')
  for yi = -5:0.5:5
    xwind = 10*sin(yi);
    ywind = 0;
    zwind = 0;
    pos = [0, yi, 0];
    force = [xwind, ywind, zwind];
    %lcmgl.drawVector3d([0,0,0],[1,1,1]);
    lcmgl.drawVector3d(pos,force);
  end
elseif strcmp(windfield, 'thermals')
  for yi = -5:0.2:5
    for xi = 1:0.2:10
    xwind = 0;
    ywind = 0;
    if (xi - 3)^2 + yi^2 < 1
      zwind = 3;
    else
      zwind = 0;
    end
    pos = [xi, yi, 0];
    force = [xwind, ywind, zwind];
    lcmgl.drawVector3d(pos,force);
    end
  end
else
  for xi = 1:10
    %for yi = 1:10
    for zi = 1:10
      
      xwind = 0;
      
      if strcmp(windfield, 'zero')
        ywind = 0;
      elseif strcmp(windfield, 'constant')
        ywind = 5;
      elseif strcmp(windfield, 'linear')
        ywind = zi;
      elseif strcmp(windfield, 'quadratic')
        ywind = zi^2;
      elseif strcmp(windfield, 'sqrt')
        ywind = (abs(zquad))^(1/2);
      elseif strcmp(windfield, 'exp')
        a = 1;
        C=20/exp(6.5);
        b=-1;
        d = 0;
        %z  = b + C*exp(a*ydotdot);
        ywind = 1/a*log((zi-b)/C) - d;
      elseif strcmp(windfield, 'tvsin')
        ywind = -10*sin(10*mytime);
      elseif strcmp(windfield, 'tlinear')
        ywind = -5 - mytime;
      else
        disp('Please specify which kind of wind field!')
      end
      
      zwind = 0;
      
      pos = [xi, 0, zi];
      force = [xwind, ywind, zwind];
      %lcmgl.drawVector3d([0,0,0],[1,1,1]);
      lcmgl.drawVector3d(pos,force);
    end
    
  end
end


%lcmgl.glColor3f(0, 0, 1);
%lcmgl.plot3(x(1,1:2)+1,x(2,1:2),x(3,1:2));
lcmgl.switchBuffers;


end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
%g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = finalCost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))]; % original
%dh = [1,zeros(1,size(x,1)-1),1]; % is this right?

end

