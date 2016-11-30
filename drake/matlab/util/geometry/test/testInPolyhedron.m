function testInPolyhedron()
verts = repmat([0.08;0.05;0.02],1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];
pt = [[0;0;0] [0;0;-0.02] [0;0;1]];
flag = inPolyhedron(verts,pt);
if(any(flag~=[true true false]))
  error('inPolyhedron is not correct');
end
verts = repmat([0.08;0.05;0],1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];
pt = [[0.01;0.03;0] [0.01;0.03;0.01]];
flag = inPolyhedron(verts,pt);
if(any(flag~=[true false]))
  error('inPolyhedron is not correct');
end
end