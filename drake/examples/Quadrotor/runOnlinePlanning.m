function runOnlinePlanning
% NOTEST
    load 'funLib.mat'

    r = Quadrotor();
    r = r.addTree([0.2,0.2,2], [2;0], 0);
    r = r.addTree([0.2,0.2,2], [3;-0.5], 0);
    r = r.addTree([0.2,0.2,2], [4.5;-1], 0);
    r = r.addTree([0.2,0.2,2], [4.5;-2], 0);
    r = r.addTree([0.2,0.2,2], [5;0], 0);

    v = r.constructVisualizer();

    collisionGeometry = r.body(1).collision_geometry(:);
    forest = {};

    for i = 1:numel(collisionGeometry);
        name = collisionGeometry{i}.name;
        if strcmp(name, 'treeTrunk') ~= 0
            forest{end+1} = collisionGeometry{i}.getPoints();
        end

    end

    x0 = funLib.funnels{1}.x0(:,1);
    x0(3) = 1; 

    op = OnlinePlannerMex({funLib.funnels{[2,4,5,1,3]}}, forest, 1, [1;2;3], 1.1702, 0, x0, 4);

    op = op.setInputFrame(r.getStateFrame);
    op = op.setOutputFrame(r.getInputFrame);

    sysCl = feedback(r, op);
    
    sysCl = sysCl.setSimulinkParam('Solver', 'ode1');
    sysCl = sysCl.setSimulinkParam('FixedStep', '0.001');    
    
    xtrajSim = sysCl.simulate([0 10], x0);

    v.playback(xtrajSim);
end