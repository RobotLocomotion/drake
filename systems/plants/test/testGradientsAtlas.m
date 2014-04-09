function testGradientsAtlas()
r = createAtlas('rpy');
nq = r.num_q;
nv = r.num_q;

q = randn(nq, 1);
v = randn(nv, 1);

[H,C,B,dH,dC,dB] = manipulatorDynamics(r,q,v,false);

end