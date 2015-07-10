r = Quadrotor();
r = r.addTree([0.3,0.3,2], [2;0], 0);
v = r.constructVisualizer();

collisionGeometry = r.body(1).collision_geometry(:);
forest = {};

for i = 1:numel(collisionGeometry);
    name = collisionGeometry{i}.name;
    if strcmp(name, 'treeTrunk') ~= 0
        forest{end+1} = collisionGeometry{i}.getPoints();
    end
    
end

load('funLib.mat');

x0 = funLib.funnels{1}.x0(:,1);

op = OnlinePlannerMex(funLib.funnels, forest, 1, [1;2;3], 1.1702, 0, x0, 4);

op = op.setInputFrame(r.getStateFrame);
op = op.setOutputFrame(r.getInputFrame);

sysCl = feedback(r, op);

xtrajSim = simulateEuler(sysCl, 1.1702, x0, 0.001);