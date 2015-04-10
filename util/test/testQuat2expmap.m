function testQuat2expmap
q = uniformlyRandomQuat();
test_userfun(q);

q = [1;0;0;0];
test_userfun(q);

q = [-1;0;0;0];
test_userfun(q);
end

function test_userfun(q)
[w,dw] = quat2expmap(q);
[w_mex,dw_mex] = quat2expmapmex(q);
valuecheck(w,w_mex);
valuecheck(dw,dw_mex);
a = quat2axis(q);
valuecheck(a(1:3)*a(4),w);
[q_back,dq,ddq] = expmap2quat(w);
valuecheck(abs(q'*q_back),1,1e-5);
valuecheck(dw*dq,eye(3),1e-5);
end