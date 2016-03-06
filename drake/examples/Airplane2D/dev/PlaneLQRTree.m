function c=PlaneLQRTree(p)

if (nargin<1)
  p=PlanePlant();
end

xG= [4; 9; 0; 0];
uG=0;
Q = diag([1 1 10 .1]); %diag([10 10 1 .1]);
R=.01;
Qf = diag([1 1 10 .1]); %diag([1 1 10 10]);

% add obstacles
con=struct();

disp('Adding obstacles...');
field = ObstacleField();
field.GenerateRandomObstacles();
con = field.AddConstraints(con);

figure(25); clf;  hold on;
v = PlaneVisualizer(field);
v.draw(0,xG);
drawnow


%options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,[1 2]);  % for debugging
options.num_branches=5;
options.verify=false;
options.stabilize_goal=false;
options.con=con;
options.xs = [4.5; 0; 0; 0];
options.Tslb = .2;
options.Tsub = 3;

px = p.getStateFrame.getPoly;
options.Vf = QuadraticLyapunovFunction(p.getStateFrame,1e4*Qf);
c = LQRTree.buildLQRTree(p,xG,uG,@()rand(4,1).*[6;6;pi/2;4]-[-2;0;pi/4;2],Q,R,options);

end

      function [J,dJ]=plotDircolTraj(t,x,u,plotdims)
        figure(25);
        h=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
        drawnow;
        delete(h);
        J=0;
        dJ=[0*t(:);0*x(:);0*u(:)]';
      end

