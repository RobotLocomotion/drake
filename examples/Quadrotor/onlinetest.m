r = Quadrotor();
r = r.addTree([0.3,0.3,2], [2;0], 0);
v = r.constructVisualizer();

%Q = diag([10*ones(6,1);ones(6,1)]);
%R = 0.1*eye(4);
%Qf = diag([100*ones(6,1);ones(6,1)]);    
%load('trajectoryLibrary.mat');
%trajLib = trajLib.setupFrames(r);
%trajLib = trajLib.stabilizeTrajectories(r, Q, R, Qf);

collisionGeometry = r.body(1).collision_geometry(:);
forest = {};

for i = 1:numel(collisionGeometry);
    name = collisionGeometry{i}.name;
    if strcmp(name, 'treeTrunk') ~= 0
        forest{end+1} = collisionGeometry{i}.getPoints();
    end
    
end

load('funnel_library.mat');


x0 = funLib.funnels{1}.x0(:,1);
x0(3) = 0.5;

op = OnlinePlannerMex(funLib.funnels, forest, 1, [1;2;3], 0.5, 0, x0, 4);

op = op.setInputFrame(r.getStateFrame);
op = op.setOutputFrame(r.getInputFrame);

sysCl = feedback(r, op);

xtrajSim = simulateEuler(sysCl, 0.8, x0, 0.001);