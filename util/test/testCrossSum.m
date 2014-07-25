function testCrossSum
r1 = randn(3,1);
r2 = randn(3,1);
[c,dc] = crossSum(r1,r2);
valuecheck(cross(r1,r2),c);
[~,dc_numeric] = geval(@(x) crossSum(x(1:3),x(4:6)),[r1;r2],struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-3);
r1 = randn(3,5);
r2 = randn(3,5);
[c,dc] = crossSum(r1,r2);
valuecheck(c,sum(cross(r1,r2,1),2));
[~,dc_numeric] = geval(@(x) crossSum(reshape(x(1:15),3,5),reshape(x(16:30),3,5)),[reshape(r1,[],1);reshape(r2,[],1)],struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-3);
end