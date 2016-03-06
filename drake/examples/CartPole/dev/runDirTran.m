function runDirTran

%NOTEST
sys = CartPolePlant();

x0 = zeros(4,1); tf0 = 4;
utraj0 = PPTrajectory(zoh(linspace(0,tf0,51),randn(1,51)));
options = struct('maxDT',0.05,'Tmin',2,'Tmax',6,'xf',[0;pi;0;0]);%,'bGradTest',true);
[obj.xtraj,obj.utraj,info] = dirtran(sys,@cost,@finalcost,x0,utraj0,options);
if (info~=1) error('dirtran failed to find a trajectory'); end

end


function [g,dg] = cost(t,x,u)
  xd = repmat([0;pi;0;0],1,size(x,2));
  x(2,:) = mod(x(2,:),2*pi);
  xerr = x - repmat(xd,1,size(x,2));
  Q = diag([10,10,1,1]);  R=1;
  
  g = sum((Q*xerr).*xerr,1) + (R*u).*u;

  if (nargout>1)
    dgdt = 0;
    dgdx = 2*xerr'*Q;
    dgdu = 2*u'*R;
    dg{1} = [dgdt,dgdx,dgdu];
  end
end
  
function [h,dh] = finalcost(t,x)
  xd = repmat([0;pi;0;0],1,size(x,2));
  x(2,:) = mod(x(2,:),2*pi);
  xerr = x - repmat(xd,1,size(x,2));
  Qf = diag([10,10,1,1]);
  h = sum((Qf*xerr).*xerr,1);

  if (nargout>1)
    dhdt = 0;
    dhdx = 2*xerr'*Qf;
    dh{1} = [dhdt,dhdx];
  end
end
