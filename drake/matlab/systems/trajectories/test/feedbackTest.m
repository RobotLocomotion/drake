function feedbackTest()
num_pieces = 2; % set to 1 and set order to 1 for a segfault
order = 1; % set to 1 to get nx x 1 state matrix for the closed loop system

nx = 4;
nu = 2;
ny = 4;

A = randn(nx, nx);
B = randn(nx, nu);
C = randn(ny, nx);
D = zeros(ny, nu);
p = LinearSystem(A,B,[],[],C,D);

Ccon = ConstantTrajectory(zeros(nu, 0));

t = linspace(0, 1, num_pieces + 1);
Dcon = PPTrajectory(mkpp(t,randn((length(t) - 1) * nu * ny, order),[nu, ny]));
controller = LinearSystem([], [], [], [], Ccon, Dcon);

controller = controller.setOutputFrame(p.getInputFrame);
controller = controller.setInputFrame(p.getOutputFrame);
sys_cl = feedback(p,controller);

end
