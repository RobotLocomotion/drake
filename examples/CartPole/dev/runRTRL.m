function [utraj,xtraj,info]=runRTRL(xInit,xDes,utraj0)

% NOTEST
if (nargin<3) utraj0=PPTrajectory(foh(linspace(0,4,31),randn(1,31)));end
if (nargin<2) xDes=[0;pi;0;0]; end
if (nargin<1) xInit=zeros(4,1); end
  


cp=CartPolePlant;
%con.xf=[];
con.xf.ub=xDes;
con.xf.lb=xDes;
con.tf.ub=6;
con.tf.lb=2;
con.nKnotsInSubTraj=10;%Using single shooting

options=struct('method','rtrl');
tic
[utraj,xtraj,info]=trajectoryOptimization(cp,@cost,@finalCost,xInit,utraj0,con,options);
toc
if(info~=1) warning('failed to find a trajectory'); end
c = tvlqr(cp,xtraj,utraj,eye(4),1,eye(4));

t = xtraj.getBreaks();
v=CartPoleVisualizer();
v.playback(xtraj);
%{
    function f=stateConstraintFun(x)
        f=x(end-3:end,1);
    end
%}
function [g,dg]=cost(t,x,u)
R = 1;
g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
end
function [h,dh]=finalCost(t,x)
h=0;
dh=[0,zeros(1,length(x))];
end
end