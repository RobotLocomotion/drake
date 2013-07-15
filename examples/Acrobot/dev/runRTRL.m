function [xtraj,utraj,info]=runRTRL(xInit,xDes)
%Do not try the swing up case, due to the ill conditioning of the Hessian
%matrix, this takes extremly long time to swing up!!!!!

% NOTEST

if (nargin<2) xDes=[pi;0;0;0]; end
if (nargin<1) xInit=xDes + 0.1*randn(4,1); end
  

tf=4;
acrobot=AcrobotPlant;

utraj0=PPTrajectory(zoh(linspace(0,tf,401),[randn(1,400) 0]));

con.xf.Aeq=eye(4);
con.xf.beq=xDes;

options={};
tic
[utraj,xtraj,info]=trajectoryOptimization(acrobot,@rtrl_snopt_transcription,@cost,@finalCost,xInit,utraj0,con,options);
toc
c = tvlqr(acrobot,xtraj,utraj,eye(4),1,eye(4));

t = xtraj.getBreaks();
t = linspace(t(1),t(end),401);
v=AcrobotVisualizer();
v.playback(xtraj);
%{
    function f=stateConstraintFun(x)
        f=x(end-3:end,1);
    end
%}
function [g,dg]=cost(t,x,u)
xerr=x-[pi;0;0;0];
xerr(1,:)=mod(xerr(1,:)+pi,2*pi)-pi;
xerr(2,:)=mod(xerr(2,:)+pi,2*pi)-pi;
Q=diag([0 0 0 0]);
R=1;
g=0.5*xerr'*Q*xerr+0.5*u'*R*u;
dgdt=0;
dgdx=xerr'*Q;
dgdu=u'*R;
dg{1}=[dgdt dgdx dgdu];
end
function [h,dh]=finalCost(t,x)
xerr=x-[pi;0;0;0];
xerr(1,:)=mod(xerr(1,:)+pi,2*pi)-pi;
xerr(2,:)=mod(xerr(2,:)+pi,2*pi)-pi;
Qf=diag([0 0 0 0]);
h=0.5*xerr'*Qf*xerr;
dhdx=xerr'*Qf;
dhdt=0;
dh{1}=[dhdt dhdx];
end
end