function testContactsPlanarBlock
    options = [];
    options.floating = true;
    options.terrain = RigidBodyFlatTerrain();
    options.twoD = true;
    %options.use_bullet = false;
    p = TimeSteppingRigidBodyManipulator('brick1.urdf', 0.01, options);
    p.getNumPositions
    v = p.constructVisualizer;
    x0 = [0,2,30*pi/180,0,0,0]';
    xtraj = p.simulate([0 10], x0)
    v.playback(xtraj)
end

