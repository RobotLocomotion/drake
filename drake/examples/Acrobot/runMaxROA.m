function runMaxROA

p = AcrobotPlant;

% Set input limits
%p = setInputLimits(p,-20,20);
p = setInputLimits(p,-inf,inf);

% Do lqr
[c,V] = balanceLQR(p);
V = extractQuadraticLyapunovFunction(V);
clf;

% Do Taylor expansions
x0 = Point(p.getStateFrame,[pi;0;0;0]);
u0 = Point(p.getInputFrame,0);
sys = taylorApprox(p,0,x0,u0,3);

% Options for control design
options = struct();
options.controller_deg = 1; % Degree of polynomial controller to search for
options.max_iterations = 10; % Maximum number of iterations (3 steps per iteration)
options.converged_tol = 1e-3; % Tolerance for checking convergence

options.rho0 = 0.001; % Initial guess for rho

options.clean_tol = 1e-6; % tolerance for cleaning small terms
options.backoff_percent = 5; % 1 percent backing off

options.degL1 = options.controller_deg + 1; % Chosen to do degree matching
options.degLu = options.controller_deg - 1; 
options.degLu1 = 2; 
options.degLu2 = 2;
options.degLup = 2;
options.degLum = 2; 

% Perform controller design and verification
disp('Starting verification...')
[c,Vmax] = maxROAFeedback(sys,c,V,options);

figure(1); clf;
options.color = 'b';
plotFunnel(Vmax,options);  % theta1 vs theta2
xlabel('theta1');
ylabel('theta2');
hold on;
options.color = 'r';
plotFunnel(V,options);

V = Vmax;

% V = V.inFrame(sys.getStateFrame());

% Test stuff
% Create closed loop system with optimized controller
sysCl = feedback(p,c);
V0 = V.getPoly;
xinit = getLevelSet(decomp(V0),V0,struct('x0',[0;0;0;0]));

v = AcrobotVisualizer(p);

for k = 1:5
    x0 = 0.95*xinit(:,k) + [pi;0;0;0];
    xsim = sysCl.simulate([0 3],x0);
    xf = xsim.eval(3);
    v.playback(xsim);
    if norm(xf-[pi;0;0;0]) > 0.1
      error('Did not stabilize point inside funnel');
    end
end







