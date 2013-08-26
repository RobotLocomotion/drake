function [p, v, x0, xtraj, utraj] = planeOpt(utraj0)
%Beginnings of file to run trajectory optimization for the MURI Plane
%  Note: does not work.  Also takes a terribly long time to run.  Much to
%  be improved, there is. --Yoda

disp('constructing a Plane')
options.floating = true;
p = RigidBodyManipulator('Plane.URDF', options);

%initial u-tape
tf = 1;
pts = 10;
if (nargin<1)
    u0 = zeros(5,pts);
    u0(2,:) = 30;
    utraj0 = PPTrajectory(foh(linspace(0,tf,pts),u0));
    utraj0 = setOutputFrame(utraj0, p.getInputFrame);
end

%     [X  Y  Z  Rx Ry Rz LW  RW  EL Rd Vx Vy Vz Vr Vp Vy VLW VRW Vel Vru
x0 =  [0  0  0  0  0  0  0   0   0  0  10  0  0  0  0  0  0   0   0  0]';
xf_lb=[9 -.2 0 -.1 0 -.1 -.2 -.2 0 -.2  9 -.5 0 -1 -1 -1 -1  -1  -1 -1]';
xf_ub=[11 .2 .2 .1 .1 .1 .2  .2  .2 .2 11 .5  2  1  1  1  1   1   1  1]';
%       LW  RW  Ele Rud Thr
u_lb = [-.3 -.3 -.8 -.8 0]';
u_ub = [.3  .3  .8  .8  305]';
disp('setting up constraints')
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf_lb;
con.xf.ub = xf_ub;
con.T.lb = 0.1;   
con.T.ub = 3;
con.u.lb = u_lb;
con.u.ub = u_ub;
options.optimizer='dircol';
options.grad_method = 'numerical';
options.MajorOptimalityTolerance=.1; % 0.3
options.MinorOptimalityTolerance=.1;
options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u);
tic
disp('Starting Trajectory Optimization');
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
disp('Trajectory Optimization Finished');
toc

v = p.constructVisualizer;
v.playback_speed = .5;
v.playback(xtraj);

end
function [g, dg] = cost(t,x,u)
    g = abs(x(3))  + abs(x(4)) + abs(x(5)) + abs(x(6)) + u(2)/50;
    %g = x(3)+pi/2 - x(1);
    dg = zeros(1, 1 + size(x,1) + size(u,1));
end

function [h, dh] = finalcost(t,x)
    h = 5*abs(x(3))  + abs(x(4)) + abs(x(5)) + abs(x(6));
    %h = x(3)+pi/2 - x(1);
    dh = [1,zeros(1,size(x,1))];
end

function [J,dJ]=plotDircolTraj(t,x,u)
    
    %{
    
    figure(25);
    
    clf;
    h=plot(x(1,:),x(3,:),'r.-');
    %x =    [X   Z  Pitch    Elev  xdot zdot pitchdot elevdot]
    %xf_lb = [-.2 -3 -pi/2-.1 -pi/3 -.02 -4   -1       -1]';
    %xf_ub = [0   0  -pi/2+.1 pi/3   .02 0    1        1]';
    
    xlims = xlim;xmin = xlims(1);xmax = xlims(2);
    ylims = ylim;ymin = ylims(1);ymax = ylims(2);
    yinc = (ymax-ymin)/8;
    xinc = (xmax-xmin)/8;
    if (x(4,end)>pi/4)
        color4 = 'red';
    else
        color4 = 'green';
    end
    if (x(3,end)<-1.5)
        color3 = 'red';
    else
        color3 = 'green';
    end
    if (x(6,end)<-2.5)
        color6 = 'red';
    else
        color6 = 'green';
    end
    %Three rectangles are pitch, elev, and zdot.
    rectangle('Position',[xmin,ymin+yinc+min(0,yinc*x(3,end)*2/(pi)),xinc,yinc*abs(x(3,end)*2/(pi))+.00001], 'FaceColor', color3);
    rectangle('Position',[xmin+xinc,ymin+yinc+min(0,yinc*x(4,end)*2/(2*pi/3)),xinc,yinc*abs(x(4,end)*2/(2*pi/3))+.00001], 'FaceColor', color4);
    rectangle('Position',[xmin+2*xinc,ymin+yinc+min(0,yinc*x(6,end)*2/8),xinc,yinc*abs(x(6,end)*2/8)+.00001], 'FaceColor', color6);
    
    line([xmin xmax], [ymin+yinc, ymin+yinc]);
    
    drawnow;
    delete(h);
%}
    J=0;
    dJ=[0*t(:);0*x(:);0*u(:)]';

end