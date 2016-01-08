function testChangeRootLink

options.floating = false;
r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);
r = r.changeRootLink('r_foot',zeros(3,1),zeros(3,1));

v = r.constructVisualizer();
v.inspector();
