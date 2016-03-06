function testQuatRotateVec()
  options.grad_method = {'user','taylorvar'};
  for i=1:100
    q = rand(4,1); q = q/norm(q);
    r = rand(3,1);
    [~,~] = geval(@quatRotateVec,q,r,options);
    valuecheck(max(abs(quatRotateVec(q,r)-quat2rotmat(q)*r)),0,1e-6);
  end
end

%function [q,r] = makeQuatAndVec(x)
  %q = rpy2quat(x);
  %r = magic(3)*x;
%end

%function [r_rot,dr_rot] = gradTestFun(x)
  %if nargout > 1
    %[q,r,dq,dr] = geval(2,@makeQuatAndVec,x,struct('grad_method','taylorvar'));
    %[r_rot, dr_rot] = quatRotateVec(q,r,dq,dr);
  %else
    %[q,r] = makeQuatAndVec(x);
    %r_rot = quatRotateVec(q,r);
  %end
%end

%function err = valTestFun(x)
    %[q,r] = makeQuatAndVec(x);
    %err = max(abs(quatRotateVec(q,r)-rpy2rotmat(x)*r));
%end
