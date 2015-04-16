function testFlipExpmap
for i = 1:100
quat = uniformlyRandomQuat();
expmap = quat2expmap(quat);
options.grad_method = {'taylorvar','user'};
[w_flip,dw_flip] = geval(@flipExpmap,expmap,options);
quat2 = expmap2quat(w_flip);
valuecheck(abs(quat2'*quat),1,1e-5);
end
end