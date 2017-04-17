function xtraj = test_elastic(restitution)
    options.terrain = RigidBodyFlatTerrain();
    options.floating = true;
    options.dt = 0.001;
    options.use_bullet = false;
    options.enable_fastqp = false;
    options.ignore_self_collisions = true;
    options.restitution = restitution;

    w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
    urdf = fullfile('Planar_CBSE_Window.URDF');

    G = -10;
    p = PlanarRigidBodyManipulator(urdf, options);
    p = p.setGravity([0; 0; G]);
    r = TimeSteppingRigidBodyManipulator(p, options.dt, options);

    tf = 2;
    x0 = [-1.8; 1; pi/3; 0; 0; 0];

    p = p.compile();

    [states, times] = elasticLCP(p, x0, tf, options.restitution);

    xtraj_elastic = PPTrajectory(foh(times, states));
    v = r.constructVisualizer();
    xtraj_elastic = xtraj_elastic.setOutputFrame(r.getStateFrame);
    v.playback(xtraj_elastic, struct('slider', true));
end