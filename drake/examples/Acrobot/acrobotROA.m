clear all; close all;

% Acrobot model
p = PlanarRigidBodyManipulator('Acrobot.urdf');

% Fixed point (upright configuration)
x0 = Point(p.getStateFrame,[pi;0;0;0]);
u0 = Point(p.getInputFrame,zeros(1,1));

% LQR controller
Q = diag([10;10;1;1]);
R = 0.1;
[c,V0] = tilqr(p,x0,u0,Q,R);

% Construct closed loop system
sys=feedback(p,c);

disp('Doing taylor approx...');
psys = taylorApprox(sys,0,x0,u0,3);  % Taylor approximate the dynamics to degree 3 

%% FILL THIS IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options.degV = 2; % Set degree of Lyapunov function 
options.degL1 = 2; % Set degrees of multiplier polynomials

options.method = 'bilinear'; % Method for computing ROA (see regionOfAttraction.m comments)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


options.converged_tol = 1e-7;
options.max_iterations = 15;

% Multiply V0 by 10 to initialize binary search with a small ROA
V0 = V0*10;

disp('Starting SOS verification...');
% V = regionOfAttraction_old(psys,V0,options); V.Vpoly
Vroa = regionOfAttraction(psys,V0,options); 

% Print the polynomial V
disp(' ');
disp('V_grade (Lyapunov function):');
V_grade = Vroa.Vpoly


% Shift the coordinates of V back so that 0 corresponds to the origin of
% the Acrobot instead of the upright position
Vroa = Vroa.inFrame(p.getStateFrame);
x = p.getStateFrame.getPoly;
V = Vroa.Vpoly;

% Plot V
close all;
plotFunnel(Vroa);
title('Verified ROA');

%% FILL THIS IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = [2.938,0.45,0,0]; %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

simulate = true; % Set this to true when you want to simulate

if simulate
    
    xtraj = sys.simulate([0 5],x0);
    v = p.constructVisualizer;
    v.axis = [-4 4 -4 4];
    playback(v,xtraj);
    t_grade = linspace(0,5,100);
    x_grade = xtraj.eval(t_grade);
    x_grade = x_grade';
    
    Vs = [];
    for k = 1:length(x_grade)
        Vs = [Vs, double(subs(V,x,x_grade(k,:)'))];
    end
    
    figure(2)
    plot(t_grade,Vs);
    title('V vs t')
    
end

% Format long
format long