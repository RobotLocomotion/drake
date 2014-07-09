function testDoKinematicsMex()

floatingType = 'rpy';
r = createAtlas(floatingType);
q = getRandomConfiguration(r);
kinsol = doKinematics(r, q, false, true);

end