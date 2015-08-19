function basicMultiFrameTest

f1 = CoordinateFrame('test1',3,'a');
f2 = CoordinateFrame('test2',2,'b');

mf = MultiCoordinateFrame({f1,f2});

p = Point(mf,(1:5)');
valuecheck(p.a1,1);
valuecheck(p.a2,2);
valuecheck(p.a3,3);
valuecheck(p.b1,4);
valuecheck(p.b2,5);

drawFrameGraph(mf);