function testParseBodyOrFrameID()
  options.floating = true;
  r = RigidBodyManipulator('FallingBrick.urdf',options);
  r = r.addFrame(RigidBodyFrame(2,[0;0;0],[0;0;0],'test_frame'));
  valuecheck(r.parseBodyOrFrameID(1),1);
  valuecheck(r.parseBodyOrFrameID('brick'),2);
  valuecheck(r.parseBodyOrFrameID(-1),-1);
  valuecheck(r.parseBodyOrFrameID('test_frame'),-1);
end
