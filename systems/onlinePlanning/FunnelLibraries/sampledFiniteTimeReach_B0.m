function [V,rho,Phi] = sampledFiniteTimeReach_B0(sys,polyOrig,Vtraj0,G,R0,tv,ts,xtraj0,utraj,options,Phi,rho)
% % Implements time-varying reachability computation.
% The approach searches for a Lyapunov function
% and attempts to minimize the size of the funnel.
%
% @param sys Taylor expanded closed loop system (with tvlqr)
% @param polyOrig Taylor expanded original system (not closed loop)
% @param Vtraj0 Initial guess for Lyapunov function
% @param G Target set description
% @param tv Tvlqr controller
% @param ts Time samples
% @param xtraj0 Nominal trajectory
% @param utraj Nominal control input
% @param options Options structure
%
% @option controller_deg Degree of controller to search for
% @option max_iterations Maximum number of iterations (3 steps per
% iteration)
% @option converged_tol Tolerance for convergence
% @option saturations True if input saturations exist, False if not
% @option rho0 Initial guess for rho
% @option clean_tol Tolerance for cleaning small terms
% @option backoff_percent Percentage to back off on objective (helps with
% numerics)
% @option degL1 Multiplier degree
% @option Lu Multiplier degree
% @option Lu1 Multiplier degree
% @option Lu2 Multiplier degree
% @option Lup Multiplier degree
% @option Lum Multiplier degree

addpath_mosek;

% Get the necessary variables
t=msspoly('t',1);
x=Vtraj0.getFrame.getPoly;
ts=ts(:);

num_x = sys.getNumStates();
num_xd = sys.getNumDiscStates();
num_xc = sys.getNumContStates();
if (num_xd), xd = x(1:num_xd); x = x(num_xd + (1:num_xc)); end
num_u = sys.getNumInputs();

if options.saturations && (num_u > 1)
    error('Sorry, I cannot handle actuator saturations for systems with more than one actuator.')
end

u = polyOrig.getInputFrame.getPoly;

% Default options
if (nargin<10) options = struct(); end
if (~isfield(options,'controller_deg')) options.controller_deg = 1; end % Degree of polynomial controller to search for
if (~isfield(options,'max_iterations')) options.max_iterations = 10; end % Maximum number of iterations (3 steps per iteration)
if (~isfield(options,'converged_tol')) options.converged_tol = 1e-3; end % Tolerance for checking convergence
if (~isfield(options,'saturations')) options.saturations = false; end % Set whether or not there are any input saturations
if (~isfield(options,'rho0')) options.rho0 = 0.1*ones(length(ts),1); options.rho0(end) = 1; end % Initial "guessed" rho
if (~isfield(options,'clean_tol')) options.clean_tol = 1e-6; end % tolerance for cleaning small terms
if (~isfield(options,'backoff_percent')) options.backoff_percent = 5; end % 5 percent backing off
if (~isfield(options,'degL1')) options.degL1 = options.controller_deg + 1; end % Chosen to do degree matching
if (~isfield(options,'degLu')) options.degLu = options.controller_deg - 1; end
if (~isfield(options,'degLu1')) options.degLu1 = 2; end
if (~isfield(options,'degLu2')) options.degLu2 = 2; end
if (~isfield(options,'degLup')) options.degLup = 2; end
if (~isfield(options,'degLum')) options.degLum = 2; end


% if (isnumeric(G) && ismatrix(G) && all(size(G)==[num_xc,num_xc]))
%   G = QuadraticLyapunovFunction(Vtraj0.getFrame,G);
% end
% typecheck(G,'PolynomialLyapunovFunction');
typecheck(Vtraj0,'PolynomialLyapunovFunction');


% %% for now, let's require that G matches V at the final conditions
% if (~equalpoly(clean(G.getPoly(ts(end))),clean(Vtraj0.getPoly(ts(end)))))
%   error('for now, I require that G matches V at the final conditions');
% end

N = length(ts);
Vmin = zeros(N-1,1);

xdottraj = fnder(xtraj0);
sys = sys.inStateFrame(Vtraj0.getFrame); % convert system to Lyapunov function coordinates
polyOrig = polyOrig.inStateFrame(Vtraj0.getFrame); % convert polyOrig to Lyapunov function coordinates

% evaluate dynamics and Vtraj at every ts once (for efficiency/clarity)
for i=1:N
  x0=xtraj0.eval(ts(i)); x0=x0(num_xd+(1:num_xc));

  fy{i} = sys.getPolyDynamics(ts(i));
  if (sys.getNumInputs>0)   % zero all inputs
    fy{i} = subs(fy{i},sys.getInputFrame.getPoly,zeros(sys.getNumInputs,1));
  end

  forig_u{i} = polyOrig.p_dynamics_traj.eval(ts(i));

  if options.saturations
      forig_umax{i} = subss(polyOrig.p_dynamics_traj.eval(ts(i)),u,options.umax);
      forig_umin{i} = subss(polyOrig.p_dynamics_traj.eval(ts(i)),u,options.umin);
  end

  K = tv.D.eval(ts(i));
  ui{i} = utraj.eval(ts(i)) + K*x;

  % Initialize Phi
  if nargin < 11
      if i > 1
          Phi{i} = 0.5*eye(length(x));
      else
          Phi{i} = zeros(length(x),length(x));
      end
  end



  Vy{i} = Vtraj0.getPoly(ts(i)) + x'*Phi{i}*x;

  Vmin(i) =  minimumV(x,Vy{i});

  i


end


% sys = sys.inStateFrame(Vtraj0.getFrame);

% Initialize rho with "tube" constraints
% rho = initializeRhoSPOT(V,Vdot,dts,options,x);

save inside_verification0.mat x Vtraj0 Vy ts forig_u options u ui x utraj forig_u Vmin sys Phi

dts = diff(ts);

% options.max_iterations = 0;

% options.rho0 = 30/10000; % 30/10000
% options.rho0 = 0.01; % % 0.005

% options.rho0_tau = 10; % 2.2;
options.converged_tol = -Inf;

% Initialize rho
if nargin < 12
    % rho = exp(options.rho0_tau*(ts-ts(1))/(ts(end)-ts(1)))-1+options.rho0; %+max(Vmin);
    % [Vy,rho,Phi] = rhoAndVLineSearch(Vtraj0,Vy,utraj,ts,forig_u,Phi,dts,options,u,ui,x,sys);
    rho = rhoLineSearch(Vtraj0,Vy,utraj,ts,forig_u,Phi,dts,options,u,ui,x,sys);
end

rhodot = diff(rho)./dts;

% Phase I: Find initial condition set
 save inside_verification1.mat x Vtraj0 Vy ts forig_u Phi options u ui x utraj forig_u Vmin sys rho rhodot dts
%load inside_verification1.mat

G_end = (Vtraj0.S.eval(ts(end)) + Phi{end})/rho(end);
G_end = G_end/5; % /2

% G_end = (Vtraj0.S.eval(ts(end)) + options.Phi_end_old)/options.rho_end_old;
% G_end = G_end/5;

% rho_fac = 0.2;
[rho_fac,L_tau] = computeRhoFac(Vy{1},G,rho(1),x);
rho_fac
L_tau
while rho_fac < 1.00 % 0.35 % 0.40 % 0.50 
        
    if options.saturations
        
        error('Does not work with saturations yet. easy fix');
        
        % First step: Fix rho and V, search for L
        [L1,Lu1,Lu2,Lp,Lup,Lm,Lum] = findLwithSats(Vtraj0,Vy,rho,rhodot,ts,forig_u,Phi,options,forig_umax,forig_umin,u,ui,x);
        plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
        hold on
        
        % Second step: Fix V and L, search for u and rho
        [ui,rho,Lu1f,Lu2f,ucoeffs] = findURhowithSats(Vy,Vtraj0,utraj,ts,forig_u,Phi,options,forig_umax,forig_umin,u,x,L1,Lu1,Lu2,Lp,Lup,Lm,Lum);
        rhodot = diff(rho)./dts;
        plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
        
        % Third step: Fix u and L, search for V and rho
        [Vy,rho,Phi] = findVRhowithSats(Vy,Vtraj0,ts,forig_u,Phi,options,forig_umax,forig_umin,x,u,ui,L1,Lp,Lm,Lu1f,Lu2f);
        rhodot = diff(rho)./dts;
        plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
    else
        % First step: Fix rho and V, search for L and u
        % [L1,ui] = findLU(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x);
        L1 = findL(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x);
        plot(ts,rho);
        % title(['iteration ' num2str(iter)])
        drawnow;
        hold on
        
        % Second step: Fix L, search for V and rho
        % [Vy,rho,Phi] = findVRho_containG_difference(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,rho_fac,R0);
        [Vy,rho,Phi] = findVRho_containG_fixed_end(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,G_end,L_tau);
        % [Vy,rho,Phi] = findVRho_containG_vol(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,rho_fac(k),rho);
        % [Vy,rho,Phi] = findVRho_containG_volproj(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,1,rho,'nohack');
        % [Vy,rho,Phi] = findVRho_containG_linvol(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,1,rho,'nohack');
        rhodot = diff(rho)./dts;
        plot(ts,rho);
        % title(['iteration ' num2str(iter)])
        drawnow;
        
        % Figure out what rho_fac is
        [rho_fac,L_tau] = computeRhoFac(Vy{1},G,rho(1));
        rho_fac
        L_tau
        
%        % First step: Fix rho and V, search for L and u
%         % [L1,ui] = findLU(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x);
%         L1 = findL(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x);
%         plot(ts,rho);
%         % title(['iteration ' num2str(iter)])
%         drawnow;
%         hold on
%         
%         % Second step: Fix L, search for V and rho
%         [Vy,rho,Phi] = findVRho_containG(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,rho_fac,R0);
%         % [Vy,rho,Phi] = findVRho_containG_vol(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,rho_fac(k),rho);
%         % [Vy,rho,Phi] = findVRho_containG_volproj(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,1,rho,'nohack');
%         % [Vy,rho,Phi] = findVRho_containG_linvol(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,1,rho,'nohack');
%         rhodot = diff(rho)./dts;
%         plot(ts,rho);
%         % title(['iteration ' num2str(iter)])
%         drawnow;
        

    end
    
    % Plot stuff
    for k = 1:length(ts)
        S0(:,:,k) = eval(Vtraj0.S,ts(k))/rho(k);
        s10(:,k) = eval(Vtraj0.s1,ts(k))/rho(k);
        s20(k) = eval(Vtraj0.s2,ts(k))/rho(k);
        Phik(:,:,k) = double(Phi{k})/rho(k);
        S(:,:,k) = S0(:,:,k) + Phik(:,:,k);
    end
    
    STraj = PPTrajectory(spline(ts(1:end),S));
    s1Traj = PPTrajectory(spline(ts(1:end),s10));
    s2Traj = PPTrajectory(spline(ts(1:end),s20));
    
    V = QuadraticLyapunovFunction(Vtraj0.getFrame,STraj,s1Traj,s2Traj);
    
    % Convert V back to state frame and plot it
    % Vxframe = V.inFrame(p.getStateFrame());
    figure
    options.plotdims = [1 2];
    % options.x0 = xtraj;
    options.inclusion = 'projection';
    % options.inclusion = 'slice';
    plotFunnel(V,options);
    %    fnplt(xtraj,[1 2]);
    axis equal
    drawnow;
    
    save inside_verification1_in.mat x Vtraj0 Vy ts forig_u Phi options u ui x utraj forig_u Vmin sys rho rhodot dts

    
    
end

save inside_verification2.mat x Vtraj0 Vy ts forig_u Phi options u ui x utraj forig_u Vmin sys rho rhodot dts
% load inside_verification2_in.mat

% options.max_iterations = 10; % 47; % 44
options.rho0 = rho(1);

% Now, Phase II
rhosum_last=0;
for iter=1:options.max_iterations
    if (mod(iter,60) == 0); close all; end
    if options.saturations
        
        % First step: Fix rho and V, search for L
        [L1,Lu1,Lu2,Lp,Lup,Lm,Lum] = findLwithSats(Vtraj0,Vy,rho,rhodot,ts,forig_u,Phi,options,forig_umax,forig_umin,u,ui,x);
        plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
        hold on
        
        % Second step: Fix V and L, search for u and rho
        [ui,rho,Lu1f,Lu2f,ucoeffs] = findURhowithSats(Vy,Vtraj0,utraj,ts,forig_u,Phi,options,forig_umax,forig_umin,u,x,L1,Lu1,Lu2,Lp,Lup,Lm,Lum);
        rhodot = diff(rho)./dts;
        plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
        
        % Third step: Fix u and L, search for V and rho
        [Vy,rho,Phi] = findVRhowithSats(Vy,Vtraj0,ts,forig_u,Phi,options,forig_umax,forig_umin,x,u,ui,L1,Lp,Lm,Lu1f,Lu2f);
        rhodot = diff(rho)./dts;
        plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
    else
        % First step: Fix rho and V, search for L and u
        % [L1,ui] = findLU(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x);
        L1 = findL(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x);
        %plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
        hold on
                
        % Second step: Fix L, search for V and rho
        options.rho_all = rho;
        %if iter <= 10
         [Vy,rho,Phi] = findVRho_containG_vol(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),rho,'nohack');
        %% [Vy,rho,Phi] = findVRho_containG_volproj(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),rho,'nohack'); % ,R0);
        %end
        %if (iter > 10) && (iter <= 20) 
        %% [Vy,rho,Phi] = findVRho_containG_vol(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),rho,'hack');
        %  [Vy,rho,Phi] = findVRho_containG(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),R0);
        %% [Vy,rho,Phi] = findVRho_containG_volproj(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),rho,'hack'); % ,R0);
        %end
        % if (iter > 20) && (iter <= 40) 
        %    [Vy,rho,Phi] = findVRho_containG_volproj(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),rho,'nohack'); % ,R0);    
        % end
        % if (iter > 40) && (iter <= 42)
        %     [Vy,rho,Phi] = findVRho_containG_volproj(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),rho,'hack'); % ,R0);
        % end
        
        % if (iter > 42)
        %     [Vy,rho,Phi] = findVRho_containG_volproj_fixed_end(G,Vtraj0,ts,forig_u,Phi,options,x,u,ui,L1,0.99*rho_fac(end),rho,'hack'); % ,R0);
        % end
        
        rhodot = diff(rho)./dts;
        %plot(ts,rho);
        title(['iteration ' num2str(iter)])
        drawnow;
                
    end
    
    tt = length(ts);
    for k = 1:tt % length(ts)
        S0(:,:,k) = eval(Vtraj0.S,ts(k))/rho(k);
        s10(:,k) = eval(Vtraj0.s1,ts(k))/rho(k);
        s20(k) = eval(Vtraj0.s2,ts(k))/rho(k);
        Phik(:,:,k) = double(Phi{k})/rho(k);
        S(:,:,k) = S0(:,:,k) + Phik(:,:,k);
    end
    
    STraj = PPTrajectory(spline(ts(1:tt),S));
    s1Traj = PPTrajectory(spline(ts(1:tt),s10));
    s2Traj = PPTrajectory(spline(ts(1:tt),s20));
    
    V = QuadraticLyapunovFunction(Vtraj0.getFrame,STraj,s1Traj,s2Traj);
    
    % Convert V back to state frame and plot it
    % Vxframe = V.inFrame(p.getStateFrame());
    figure
    options.plotdims = [1 2];
    % options.x0 = xtraj;
    options.inclusion = 'projection';
    % options.inclusion = 'slice';
    plotFunnel(V,options);
    %    fnplt(xtraj,[1 2]);
    axis equal
    drawnow;
    
    
    
    rhosum = sum(double(rho));
    
%     % check for convergence
%     if ((rhosum - rhosum_last) < options.converged_tol*rhosum_last)  % see if it's converged
%         break;
%     end
%     
%     rhosum_last = rhosum;
    
    save inside_verification2_in.mat x Vtraj0 Vy ts forig_u Phi options u ui x utraj forig_u Vmin sys rho rhodot dts

    
end

% uik = zeros(length(ucoeffs{1}),length(ts)-1);
% for k = 1:length(ui)
%     uik(:,k) = ucoeffs{k};
% end
% uik(:,end+1) = uik(:,end); % Is there something better I can do here?
% coeffspline = spline(ts(1:end),uik);
% monoms = monomials(x,1:options.controller_deg);
% usys = PolynomialControlSystem(num_x,num_u,xtraj0,utraj,coeffspline,monoms);
% usys = setInputFrame(usys,polyOrig.getStateFrame);
% usys = setOutputFrame(usys,polyOrig.getInputFrame);




% for k = 1:length(ts)
%  S0(:,:,k) = eval(Vtraj0.S,ts(k))/rho(k);
%  s10(:,k) = eval(Vtraj0.s1,ts(k))/rho(k);
%  s20(k) = eval(Vtraj0.s2,ts(k))/rho(k);
%  Phik(:,:,k) = double(Phi{k})/rho(k);
%  S(:,:,k) = S0(:,:,k) + Phik(:,:,k);
% end

for k = 1:length(ts)
    S0(:,:,k) = eval(Vtraj0.S,ts(k))/rho(k);
    s10(:,k) = eval(Vtraj0.s1,ts(k))/rho(k);
    s20(k) = eval(Vtraj0.s2,ts(k))/rho(k);
    Phik(:,:,k) = double(Phi{k})/rho(k);
    S(:,:,k) = S0(:,:,k) + Phik(:,:,k);
end

STraj = PPTrajectory(spline(ts(1:end),S));
s1Traj = PPTrajectory(spline(ts(1:end),s10));
s2Traj = PPTrajectory(spline(ts(1:end),s20));

V = QuadraticLyapunovFunction(Vtraj0.getFrame,STraj,s1Traj,s2Traj);

end

function [rho_fac,L_tau] = computeRhoFac(V0,G,rho0,x)

prog = spotsosprog;

[prog,rho_fac] = prog.newPos(1);

[prog,L_tau] = prog.newPos(1);
% x = decomp(V0);
prog = prog.withIndeterminate(x);
% Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% prog = prog.withSOS(rho0*rho_fac - V0 - tau*(1 - x'*G*x));
prog = prog.withSOS(-L_tau*(V0 - rho0) - rho_fac + x'*G*x);

options = spot_sdp_default_options;
options.verbose = 1;
sol = prog.minimize(-rho_fac,@spot_mosek,options);

rho_fac = double(sol.eval(rho_fac));
L_tau = sol.eval(L_tau);


end


function L1f = findL(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x)

N = length(ts)-1;
disp('Step 1: Searching for multipliers and controller...')

if (matlabpool('size')==0) matlabpool 4; end

% Optimized multipliers
L1f = cell(1,N);

parfor k = 1:N
    
    % k
    
    prog = spotsosprog; % keyboard;
    prog = prog.withIndeterminate(x);
    
    Phidotk = (Phi{k+1} - Phi{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    %     % Create u
    %     um = monomials(x,1:options.controller_deg);
    %     lu = [];
    %     for j = 1:length(utraj.eval(ts(k)))
    %         [prog,luj] = prog.newFree(length(um));
    %         % [prog,luj] = new(prog,length(um),'free');
    %         lu = [lu;luj'];
    %     end
    %     ui{k} = utraj.eval(ts(k)) + lu*um;
    
    % [prog,ui{k}] = prog.newFreePoly(um,length(utraj.eval(ts(k))));
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phi{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    V = clean(Vy{k},options.clean_tol);
    Vdoty = clean(Vdoty,options.clean_tol);
    
    % Declare multipliers
    L1m = monomials(x,0:options.degL1);
    [prog,l1] = prog.newFree(length(L1m));
    % [prog,l1] = new(prog,length(L1m),'free');
    L1 = l1'*L1m;
    
    % Create gammas
    [prog,gamma] = prog.newPos(1);
    % [prog,gamma] = new(prog,1,'pos');
    
    % Declare SOS conditions
    prog = prog.withSOS(-gamma*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot(k) + L1*(V-rho(k)));
    % prog.sos =  -gamma*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot(k) + L1*(V-rho(k));
    
    % Solve SOS program
    pars = spot_sdp_default_options();
    pars.verbose = 0;
    sol = prog.minimize(0,@spot_mosek,pars);
    
    % Optimized multipliers
    L1f{k} = sol.eval(L1);
    % uif{k} = sol.eval(ui{k});
    
    
end

end

function L1f = findLDebug(Vtraj0,Vy,rho,rhodot,utraj,ts,forig_u,Phi,options,u,ui,x)

N = length(ts)-1;
disp('Step 1: Searching for multipliers and controller...')

% if (matlabpool('size')==0) matlabpool 2; end

% Optimized multipliers
L1f = cell(1,N);

for k = 4 %1:N
    
    prog = spotsosprog; % keyboard;
    prog = prog.withIndeterminate(x);
    
    Phidotk = (Phi{k+1} - Phi{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    %     % Create u
    %     um = monomials(x,1:options.controller_deg);
    %     lu = [];
    %     for j = 1:length(utraj.eval(ts(k)))
    %         [prog,luj] = prog.newFree(length(um));
    %         % [prog,luj] = new(prog,length(um),'free');
    %         lu = [lu;luj'];
    %     end
    %     ui{k} = utraj.eval(ts(k)) + lu*um;
    
    % [prog,ui{k}] = prog.newFreePoly(um,length(utraj.eval(ts(k))));
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phi{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    V = clean(Vy{k},options.clean_tol);
    Vdoty = clean(Vdoty,options.clean_tol);
    
    % Declare multipliers
    L1m = monomials(x,0:options.degL1);
    [prog,l1] = prog.newFree(length(L1m));
    % [prog,l1] = new(prog,length(L1m),'free');
    L1 = l1'*L1m;
    
    % Create gammas
    [prog,gamma] = prog.newPos(1);
    % [prog,gamma] = new(prog,1,'pos');
    
    % Declare SOS conditions
    prog = prog.withSOS(-gamma*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot(k) + L1*(V-rho(k)));
    % prog.sos =  -gamma*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot(k) + L1*(V-rho(k));
    
    % Solve SOS program
    pars = spot_sdp_default_options();
    pars.verbose = 1;
    % sol = prog.minimize(0,@spot_mosek,pars);
    
    keyboard;
    
    % Optimized multipliers
    L1f{k} = sol.eval(L1);
    
    
    
    % Yalmip code
    x2 = sdpvar(length(x),1);
    V2 = msspoly2sdpvar(x,x2,V);
    Vdot2 = msspoly2sdpvar(x,x2,Vdoty);
    gammay = sdpvar(1,1);
    C = [gammay > 0];
    [L1y,cL1y,vL1y] = polynomial(x2,options.degL1);
    expr = -gammay*(x2'*x2)^2 - Vdot2 + rhodot(k) + L1y*(V2-rho(k));
    C = [C, sos(expr)];
    
    optionsY = sdpsettings('solver','mosek-sdp');
    diagnostics = solvesos(C,[],optionsY,[gammay;cL1y]);
    
    
end

end

function [V,rho,Phi] = findVRho_containG(G,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho_fac,R0)

N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
[prog,rho] = prog.newPos(N+1);

% % [prog,rho0] = prog.newFree(1);
% prog = prog.withPos(rho(1) - options.rho_all(1));

rhodot = msspoly('r',N);

% % OPTION 1
% [prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
% Phid{N+1} = mss_v2s(Phid{N+1});
% % Make V positive definite
% Send = Vtraj0.S.eval(ts(N+1));
% prog = prog.withPSD(Send + Phid{N+1});
% 
% % Bound Phi
% maxel = 0.001*abs(Phiold{N+1}+Send); %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send));
% prog = prog.withPos(maxel - (Phid{N+1}-Phiold{N+1}));
% prog = prog.withPos(maxel + (Phid{N+1}-Phiold{N+1}));
% 
% % Fix rho(end)
% prog = prog.withEqs(rho(end) - options.rho_all(end));

% OR OPTION 2
[prog,sc] = prog.newPos(1);
Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));
% Phid{N+1} = -Vtraj0.S.eval(ts(N+1)) + (Vtraj0.S.eval(ts(N+1)) + Phiold{N+1});


[prog,sc] = prog.newFree(1);
Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));
prog = prog.withPSD(Phid{N+1} + Vtraj0.S.eval(ts(N+1)));



% % Initialize Phid{1}
% Phid{1} = Phiold{1};

[prog,Phid{1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{1} = mss_v2s(Phid{1});

% Normalization constraint
prog = prog.withEqs(trace(Phid{1}) - trace(Phiold{1}));

% Make V positive definite
% [prog, Phislack0] = prog.newPSD(length(x));
prog = prog.withPSD(Vtraj0.S.eval(ts(1)) + Phid{1});

% Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% [prog,r0] = prog.newPos(1);
[prog,tau] = prog.newPos(1);

V{1} = Vtraj0.getPoly(ts(1)) + x'*Phid{1}*x;

% prog = prog.withSOS(rho(1)*rho_fac - V{1} - tau*(1 - x'*G*x));
prog = prog.withSOS(rho(1) - V{1} - tau*(rho_fac - x'*G*x));

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        % Normalization constraint
        prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        
        
    end
    
    
    
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phid{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    % prog.sos = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
    
    % Make V positive definite
    [prog,Phislack] = prog.newPSD(length(x));
    prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(k)) + Phid{k}));
    
%     if k > 1
%         % Make sure funnel contains some reasoable ellipsoid
%         [prog,tauk] = prog.newPos(1);
%         prog = prog.withSOS(rho(k) - V{k} - tauk*(1 - x'*R0*x));
%     end

    
end

% Solve SOS program
costfun = sum(rho(1:end)); %  + rho(end); % sum(rho);

costfun = costfun/1000; % /(100);

pars = spot_sdp_default_options();
pars.verbose = 1;

sol = prog.minimize(costfun,@spot_mosek,pars);


% sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

%     disp('Backing off now...')

    % Back off on objective
    
    costfun_opt = double(sol.eval(costfun));
    % rhosum = double(sum(sol.eval(rho)));
    % prog = prog.withPos((-(1 - options.backoff_percent/100)*costfun_opt + costfun));

    prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);

    sol = prog.minimize(0,@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 1:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end
% V{1} = Vtraj0.getPoly(ts(1));
% Phi{1} = zeros(length(x),length(x));

% r0 = double(sol.eval(r0));
% rho0 = double(sol.eval(rho0));


end

function [V,rho,Phi] = findVRho_containG_difference(G,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho_fac,R0)

N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
[prog,rho] = prog.newPos(N+1);

% [prog,rho0] = prog.newFree(1);
% prog = prog.withPos(rho(1) - rho0);

rhodot = msspoly('r',N);

% OPTION 1
% V at end
[prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{N+1} = mss_v2s(Phid{N+1});

% Normalization constraint
prog = prog.withEqs(trace(Phid{N+1}) - trace(Phiold{N+1}));

% Make V positive definite
[prog, Phislack] = prog.newPSD(length(x));
prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(N+1)) + Phid{N+1}));

% Bound Phi
Send = Vtraj0.S.eval(ts(N+1));
maxel = 0.1*abs(Phiold{N+1} + Send); %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send));
prog = prog.withPos(maxel - (Phid{N+1}-Phiold{N+1}));
prog = prog.withPos(maxel + (Phid{N+1}-Phiold{N+1}));

% % OR OPTION 2
% [prog,sc] = prog.newPos(1);
% Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));




% [prog,sc] = prog.newFree(1);
% Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));
% prog = prog.withPSD(Phid{N+1} + Vtraj0.S.eval(ts(N+1)));



% % Initialize Phid{1}
% Phid{1} = Phiold{1};

[prog,Phid{1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{1} = mss_v2s(Phid{1});

% Normalization constraint
prog = prog.withEqs(trace(Phid{1}) - trace(Phiold{1}));

% Make V positive definite
% [prog, Phislack0] = prog.newPSD(length(x));
prog = prog.withPSD(Vtraj0.S.eval(ts(1)) + Phid{1});

% Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% [prog,r0] = prog.newPos(1);
[prog,tau] = prog.newPos(1);

V{1} = Vtraj0.getPoly(ts(1)) + x'*Phid{1}*x;

% prog = prog.withSOS(rho(1)*rho_fac - V{1} - tau*(1 - x'*G*x));
prog = prog.withSOS(rho(1) - V{1} - tau*(rho_fac - x'*G*x));

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        % Normalization constraint
        prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        
        
    end
    
    
    
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phid{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    % prog.sos = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
    
    % Make V positive definite
    [prog,Phislack] = prog.newPSD(length(x));
    prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(k)) + Phid{k}));
    
%     if k > 1
%         % Make sure funnel contains some reasoable ellipsoid
%         [prog,tauk] = prog.newPos(1);
%         prog = prog.withSOS(rho(k) - V{k} - tauk*(1 - x'*R0*x));
%     end

    
end

% Solve SOS program
% costfun = (0.01*rho(end) - 100*rho(1))/1000; % Weighted combination 
costfun = 0.1*rho(end) - sum(rho(1:end-1));


% costfun = costfun*100; % /(1000);

pars = spot_sdp_default_options();
pars.verbose = 1;

sol = prog.minimize(costfun,@spot_mosek,pars);


% sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

%     disp('Backing off now...')

    % Back off on objective
    
    costfun_opt = double(sol.eval(costfun));
    % rhosum = double(sum(sol.eval(rho)));
    % prog = prog.withPos((-(1 - options.backoff_percent/100)*costfun_opt + costfun));

    prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);

    sol = prog.minimize(0,@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 1:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end
% V{1} = Vtraj0.getPoly(ts(1));
% Phi{1} = zeros(length(x),length(x));

% r0 = double(sol.eval(r0));
% rho0 = double(sol.eval(rho0));


end


function [V,rho,Phi] = findVRho_containG_fixed_end(G,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,G_end,L_tau)


N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
[prog,rho] = prog.newPos(N+1);

% prog = prog.withPos(1.0 - rho(end));

% [prog,rho0] = prog.newFree(1);
% prog = prog.withPos(rho(1) - rho0);

rhodot = msspoly('r',N);

% OPTION 1
% V at end
[prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{N+1} = mss_v2s(Phid{N+1});

% Normalization constraint
prog = prog.withEqs(trace(Phid{N+1}) - trace(Phiold{N+1}));

% Make V positive definite
[prog, Phislack] = prog.newPSD(length(x));
prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(N+1)) + Phid{N+1}));

% % Bound Phi
% Send = Vtraj0.S.eval(ts(N+1));
% maxel = 0.1*abs(Phiold{N+1} + Send); %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send));
% prog = prog.withPos(maxel - (Phid{N+1}-Phiold{N+1}));
% prog = prog.withPos(maxel + (Phid{N+1}-Phiold{N+1}));

% % OR OPTION 2
% [prog,sc] = prog.newPos(1);
% Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));


% Constrain V at end
V{N+1} = Vtraj0.getPoly(ts(N+1)) + x'*Phid{N+1}*x;
[prog,L0] = prog.newPos(1);
prog = prog.withSOS(V{N+1} - rho(end) - L0*(x'*G_end*x - 1));




% % Initialize Phid{1}
% Phid{1} = Phiold{1};

[prog,Phid{1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{1} = mss_v2s(Phid{1});

% Normalization constraint
prog = prog.withEqs(trace(Phid{1}) - trace(Phiold{1}));

% Make V positive definite
% [prog, Phislack0] = prog.newPSD(length(x));
prog = prog.withPSD(Vtraj0.S.eval(ts(1)) + Phid{1});

% Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% [prog,r0] = prog.newPos(1);
[prog,tau] = prog.newPos(1);

V{1} = Vtraj0.getPoly(ts(1)) + x'*Phid{1}*x;

% prog = prog.withSOS(rho(1) - V{1} - tau*(rho_fac - x'*G*x));
% prog = prog.withSOS(rho(1) - V{1} - (tau - x'*G*x));
prog = prog.withSOS(x'*G*x - tau - L_tau*(V{1} - rho(1)));

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        % Normalization constraint
        prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        
        
    end
    
    
    
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phid{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    % prog.sos = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
    
    % Make V positive definite
    [prog,Phislack] = prog.newPSD(length(x));
    prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(k)) + Phid{k}));
    
%     if k > 1
%         % Make sure funnel contains some reasoable ellipsoid
%         [prog,tauk] = prog.newPos(1);
%         prog = prog.withSOS(rho(k) - V{k} - tauk*(1 - x'*R0*x));
%     end

    
end

% Solve SOS program
costfun = -tau;
% costfun = -tau - 0.01*sum(rho(1:end-1));

% costfun = costfun*100; % /(1000);

pars = spot_sdp_default_options();
pars.verbose = 1;

sol = prog.minimize(costfun,@spot_mosek,pars);


% sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

%     disp('Backing off now...')

%     % Back off on objective
%     options.backoff_percent = 1;
%     costfun_opt = double(sol.eval(costfun));
%     % rhosum = double(sum(sol.eval(rho)));
%     % prog = prog.withPos((-(1 - options.backoff_percent/100)*costfun_opt + costfun));
% 
%     prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);
% 
%     sol = prog.minimize(0,@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 1:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end
% V{1} = Vtraj0.getPoly(ts(1));
% Phi{1} = zeros(length(x),length(x));

% r0 = double(sol.eval(r0));
% rho0 = double(sol.eval(rho0));


end



function [V,rho,Phi] = findVRho_containG_vol(G,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho_fac,rho,hack)

N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
% [prog,rho] = prog.newPos(N+1);

% [prog,rho0] = prog.newFree(1);
% prog = prog.withPos(rho(1) - rho0);

rhodot = msspoly(zeros(N,1));

% V at end
if ~strcmp(hack,'hack')
    [prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
    Phid{N+1} = mss_v2s(Phid{N+1});
    % Make V positive definite
    Send = Vtraj0.S.eval(ts(N+1));
    prog = prog.withPSD(Send + Phid{N+1});
    
    % Bound Phi
    maxel = 0.1*abs(Phiold{N+1}+Send); %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send)); 
    prog = prog.withPos(maxel - (Phid{N+1}-Phiold{N+1}));
    prog = prog.withPos(maxel + (Phid{N+1}-Phiold{N+1}));
else
    [prog,sc] = prog.newFree(1);
    Phid{N+1} = sc*(Vtraj0.S.eval(ts(N+1)));
    % Phid{N+1} = -Vtraj0.S.eval(ts(N+1)) + (1+sc)*(Vtraj0.S.eval(ts(N+1)) + Phiold{N+1});
    
    
    
%     [prog,sc] = prog.newPos(1);
%     Phid_sc = sc*(Vtraj0.S.eval(ts(N+1)));
%     [prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
%     
%     Phid{N+1} = mss_v2s(Phid{N+1});
%     
%     maxel = 0.1; %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send)); 
%     prog = prog.withPos(maxel - Phid{N+1});
%     prog = prog.withPos(maxel + Phid{N+1});
%     
%     Phid{N+1} = Phid{N+1} + Phid_sc;
    
    
    
    % Make V positive definite
    Send = Vtraj0.S.eval(ts(N+1));
    prog = prog.withPSD(Send + Phid{N+1});
    
    
end

% % Initialize Phid{1}
% Phid{1} = Phiold{1};

[prog,Phid{1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{1} = mss_v2s(Phid{1});

% Make V positive definite
prog = prog.withPSD(Vtraj0.S.eval(ts(1)) + Phid{1});



% Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% [prog,r0] = prog.newPos(1);
[prog,tau] = prog.newPos(1);

V{1} = Vtraj0.getPoly(ts(1)) + x'*Phid{1}*x;

prog = prog.withSOS(rho(1) - V{1} - tau*(rho_fac - x'*G*x));

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        % Make V positive definite
        prog = prog.withPSD(Vtraj0.S.eval(ts(k)) + Phid{k});
        
        
        
        %         % Normalization constraint
        %         prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        
        
        
    end
    
%     % Bound Phi
%     Sk = Vtraj0.S.eval(ts(k));    
%     maxel = 0.10*abs(Phiold{k}+Sk);
%     prog = prog.withPos(maxel - (Phid{k}-Phiold{k}));
%     prog = prog.withPos(maxel + (Phid{k}-Phiold{k}));
    
    
    
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phid{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    % prog.sos = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
    
    % [prog,Phislack] = prog.newPSD(length(x));
    % prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(k)) + Phid{k}));
    
    
end

% Solve SOS program
costfun = 0;
% [prog,costfun] = prog.newFree(1);

% keyboard;
for k = [(N+1)]
    S0k = Vtraj0.S.eval(ts(k));
    % P0k = (S0k + Phiold{k})/rho(k); % P0k = 1000*eye(12);
    Pk = (S0k + Phid{k})/rho(k);

    if k == N+1
        weight = 1;
%         ww = 1./[0.1 100 100 0.1*ones(1,9)];
%         ww = 10*ww/norm(ww);
%         W = diag(ww);
%         Pk = W*Pk*W;
    else
        weight = 0.1; % 0.1;
    end
    
    [prog,tauk] = maxdet(prog,Pk); % tauk lower bounds det of Pk
    costfun = costfun - weight*tauk;
    % prog = prog.withPos(costfun - (-tauk));
    
end

costfun = costfun/(10);

pars = spot_sdp_default_options();
pars.verbose = 1;
% pars.scale_monomials = true;

sol = prog.minimize(costfun,@spot_mosek,pars);

% keyboard;


% sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

    disp('Backing off now...')

    % Back off on objective
    % options.backoff_percent = 5;
    costfun_opt = double(sol.eval(costfun));
    % rhosum = double(sum(sol.eval(rho)));
    % prog = prog.withPos((-(1 - options.backoff_percent/100)*costfun_opt + costfun));

    prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);

    sol = prog.minimize(0,@spot_sedumi,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 1:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end
% V{1} = Vtraj0.getPoly(ts(1));
% Phi{1} = zeros(length(x),length(x));

% r0 = double(sol.eval(r0));
% rho0 = double(sol.eval(rho0));


end




function [V,rho,Phi] = findVRho_containG_linvol(G,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho_fac,rho)

N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
% [prog,rho] = prog.newPos(N+1);

% [prog,rho0] = prog.newFree(1);
% prog = prog.withPos(rho(1) - rho0);

rhodot = msspoly(zeros(N,1));

% V at end
% [prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
% Phid{N+1} = mss_v2s(Phid{N+1});
%
% % % Normalization constraint
% % prog = prog.withEqs(trace(Phid{N+1}) - trace(Phiold{N+1}));
%
% % Make V positive definite
% prog = prog.withPSD(Vtraj0.S.eval(ts(N+1)) + Phid{N+1});
% % [prog, Phislack] = prog.newPSD(length(x));
% % prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(N+1)) + Phid{N+1}));

[prog,sc] = prog.newPos(1);
Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));


% % Initialize Phid{1}
% Phid{1} = Phiold{1};

[prog,Phid{1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{1} = mss_v2s(Phid{1});

% % Normalization constraint
% prog = prog.withEqs(trace(Phid{1}) - trace(Phiold{1}));

% Make V positive definite
% [prog, Phislack0] = prog.newPSD(length(x));
prog = prog.withPSD(Vtraj0.S.eval(ts(1)) + Phid{1});

% Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% [prog,r0] = prog.newPos(1);
[prog,tau] = prog.newPos(1);

V{1} = Vtraj0.getPoly(ts(1)) + x'*Phid{1}*x;

prog = prog.withSOS(rho(1)*rho_fac - V{1} - tau*(1 - x'*G*x));

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        %         % Normalization constraint
        %         prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        
        
    end
    
    
    
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phid{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    % prog.sos = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
    
    % Make V positive definite
    prog = prog.withPSD(Vtraj0.S.eval(ts(k)) + Phid{k});
    % [prog,Phislack] = prog.newPSD(length(x));
    % prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(k)) + Phid{k}));
    
    
end

% Solve SOS program
costfun = 0;
% keyboard;
for k = 1:(N+1)
    S0k = Vtraj0.S.eval(ts(k));
    P0k = (S0k + Phiold{k})/rho(k);
    Pk = (S0k + Phid{k})/rho(k);
    costfun = costfun - trace(inv(P0k)*Pk);
end
% keyboard;
costfun = costfun/(10^3);

pars = spot_sdp_default_options();
pars.verbose = 1;
% pars.scale_monomials = true;

sol = prog.minimize(costfun,@spot_mosek,pars);


% sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

    disp('Backing off now...')

    % Back off on objective
    
    costfun_opt = double(sol.eval(costfun));
    % rhosum = double(sum(sol.eval(rho)));
    % prog = prog.withPos((-(1 - options.backoff_percent/100)*costfun_opt + costfun));

    prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);

    sol = prog.minimize(0,@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 1:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end
% V{1} = Vtraj0.getPoly(ts(1));
% Phi{1} = zeros(length(x),length(x));

% r0 = double(sol.eval(r0));
% rho0 = double(sol.eval(rho0));


end

function [V,rho,Phi] = findVRho_containG_volproj(G,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho_fac,rho,hack) % ,R0)

N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
% [prog,rho] = prog.newPos(N+1);

% [prog,rho0] = prog.newFree(1);
% prog = prog.withPos(rho(1) - rho0);

rhodot = msspoly(zeros(N,1));

% V at end
if ~strcmp(hack,'hack')
    [prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
    Phid{N+1} = mss_v2s(Phid{N+1});
    % Make V positive definite
    Send = Vtraj0.S.eval(ts(N+1));
    prog = prog.withPSD(Send + Phid{N+1});
    
    % Bound Phi % 0.1
    maxel = 0.1*abs(Phiold{N+1}+Send); %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send)); 
    prog = prog.withPos(maxel - (Phid{N+1}-Phiold{N+1}));
    prog = prog.withPos(maxel + (Phid{N+1}-Phiold{N+1}));
    
    
else
    [prog,sc] = prog.newPos(1);
    % prog = prog.withPos(sc - 0.98);
    Phid{N+1} = -Vtraj0.S.eval(ts(N+1)) + (1+sc)*(Vtraj0.S.eval(ts(N+1)) + Phiold{N+1});
    % Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));
end


% Initialize Phid{1}
Phid{1} = Phiold{1};

% [prog,Phid{1}] = prog.newFree(nchoosek(length(x)+1,2));
% Phid{1} = mss_v2s(Phid{1});
% 
% % % Normalization constraint
% % prog = prog.withEqs(trace(Phid{1}) - trace(Phiold{1}));
% 
% % Make V positive definite
% % [prog, Phislack0] = prog.newPSD(length(x));
% prog = prog.withPSD(Vtraj0.S.eval(ts(1)) + Phid{1});
% 
% % Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% % [prog,r0] = prog.newPos(1);
% [prog,tau] = prog.newPos(1);
% 
% V{1} = Vtraj0.getPoly(ts(1)) + x'*Phid{1}*x;
% 
% prog = prog.withSOS(rho(1) - V{1} - tau*(rho_fac - x'*G*x));

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        % Make V positive definite
        prog = prog.withPSD(Vtraj0.S.eval(ts(k)) + Phid{k});
        
        
        
        %         % Normalization constraint
        %         prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        

        
        
    end
    
    
    
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    xdotk = subss(forig_u{k},u,ui{k});
    % xdotk = xdotk + [zeros(7,1);w;zeros(4,1)]; % .*xdotk;
    Vdoty = diff(V0k,x)*xdotk + V0dotk + 2*x'*Phid{k}*xdotk + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    % prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    Vdotcond = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
 
    [prog,gamma] = prog.newPos(1);
    prog = prog.withSOS(-gamma*(x'*x)^(deg(Vdoty,x)/2) + Vdotcond);
   

    
%     if k > 1
%         % Make sure funnel contains some reasoable ellipsoid
%         [prog,tauk] = prog.newPos(1);
%         prog = prog.withSOS(rho(k) - V{k} - tauk*(1 - x'*R0*x));
%     end
    
end

% Solve SOS program
costfun = 0;
% [prog,costfun] = prog.newFree(1);
C = [eye(3), zeros(3,9)]; % Projection matrix
% keyboard;
for k = [N+1] % 1:(N+1)
    S0k = Vtraj0.S.eval(ts(k));
    P0k = (S0k + Phiold{k})/rho(k); % P0k = 1000*eye(12);
    Pk = (S0k + Phid{k})/rho(k);
    
%     W = diag([1 1/100 1/100 ones(1,9)]);
%     Pk = W*Pk*W;
    
    if k == N+1
        weight = 1;
    else
        weight = 1;
    end
    
    % Linearized projection volume
    costfun = costfun - weight*trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)));
    % prog = prog.withPos(costfun - (-trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)))));
    
end
% keyboard;
costfun = costfun/1000; % 100;

pars = spot_sdp_default_options();
pars.verbose = 1;
% pars.scale_monomials = true;

sol = prog.minimize(costfun,@spot_mosek,pars);

% keyboard;


% sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

    disp('Backing off now...')

    % Back off on objective
    costfun_opt = double(sol.eval(costfun));
    prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);

    sol = prog.minimize(0,@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 1:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end

% for k = 1:9
%     Phi{k} = Phiold{k};
%     V0k = Vtraj0.getPoly(ts(k));
%     V{k} = V0k + x'*Phi{k}*x;
% end


end

function [V,rho,Phi] = findVRho_containG_volproj_fixed_end(G,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho_fac,rho,hack) % ,R0)

N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
% [prog,rho] = prog.newPos(N+1);

% [prog,rho0] = prog.newFree(1);
% prog = prog.withPos(rho(1) - rho0);

rhodot = msspoly(zeros(N,1));

% V at end
% if ~strcmp(hack,'hack')
    [prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
    Phid{N+1} = mss_v2s(Phid{N+1});
    % Make V positive definite
    Send = Vtraj0.S.eval(ts(N+1));
    prog = prog.withPSD(Send + Phid{N+1});
    
    % Bound Phi % 0.1
    maxel = 0.001*abs(Phiold{N+1}+Send); %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send)); 
    prog = prog.withPos(maxel - (Phid{N+1}-Phiold{N+1}));
    prog = prog.withPos(maxel + (Phid{N+1}-Phiold{N+1}));
    
    
% else
%     [prog,sc] = prog.newPos(1);
%     % prog = prog.withPos(sc - 0.98);
%     Phid{N+1} = -Vtraj0.S.eval(ts(N+1)) + (1+sc)*(Vtraj0.S.eval(ts(N+1)) + Phiold{N+1});
%     % Phid{N+1} = sc*Vtraj0.S.eval(ts(N+1));
% end


% % Initialize Phid{1}
% Phid{1} = Phiold{1};

[prog,Phid{1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{1} = mss_v2s(Phid{1});

% Bound Phi % 0.1
maxel = 0.001; %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send));
prog = prog.withPos(maxel - (Phid{1}-Phiold{1}));
prog = prog.withPos(maxel + (Phid{1}-Phiold{1}));

% % % Normalization constraint
% % prog = prog.withEqs(trace(Phid{1}) - trace(Phiold{1}));
% 
% % Make V positive definite
% % [prog, Phislack0] = prog.newPSD(length(x));
% prog = prog.withPSD(Vtraj0.S.eval(ts(1)) + Phid{1});
% 
% % Make sure r0 sublevel set of x'*G*x is contained in Phid{1} ellipse
% % [prog,r0] = prog.newPos(1);
% [prog,tau] = prog.newPos(1);
% 
% V{1} = Vtraj0.getPoly(ts(1)) + x'*Phid{1}*x;
% 
% prog = prog.withSOS(rho(1) - V{1} - tau*(rho_fac - x'*G*x));

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        % Make V positive definite
        prog = prog.withPSD(Vtraj0.S.eval(ts(k)) + Phid{k});
        
        
        
        %         % Normalization constraint
        %         prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        
        
        % Bound Phi % 0.1
        maxel = 0.1*abs(Phiold{k}+Vtraj0.S.eval(ts(k))); %   max(0.10*abs(Phiold{N+1}+Send),0.05*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send))); % 0.1*max(max(abs(Phiold{N+1}+Send)))*ones(size(Send));
        prog = prog.withPos(maxel - (Phid{k}-Phiold{k}));
        prog = prog.withPos(maxel + (Phid{k}-Phiold{k}));
        
    end
    
    
    
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    xdotk = subss(forig_u{k},u,ui{k});
    % xdotk = xdotk + [zeros(7,1);w;zeros(4,1)]; % .*xdotk;
    Vdoty = diff(V0k,x)*xdotk + V0dotk + 2*x'*Phid{k}*xdotk + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    % prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    Vdotcond = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
 
    [prog,gamma] = prog.newPos(1);
    prog = prog.withSOS(-gamma*(x'*x)^(deg(Vdoty,x)/2) + Vdotcond);
   

    
%     if k > 1
%         % Make sure funnel contains some reasoable ellipsoid
%         [prog,tauk] = prog.newPos(1);
%         prog = prog.withSOS(rho(k) - V{k} - tauk*(1 - x'*R0*x));
%     end
    
end

% Solve SOS program
costfun = 0;
% [prog,costfun] = prog.newFree(1);
C = [eye(3), zeros(3,9)]; % Projection matrix
% keyboard;
for k = 2:N-1
    S0k = Vtraj0.S.eval(ts(k));
    P0k = (S0k + Phiold{k})/rho(k); % P0k = 1000*eye(12);
    Pk = (S0k + Phid{k})/rho(k);
    
%     W = diag([1 1/100 1/100 ones(1,9)]);
%     Pk = W*Pk*W;
    
    if k == N+1
        weight = 1;
    else
        weight = 1;
    end
    
    % Linearized projection volume
    costfun = costfun - weight*trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)));
    % prog = prog.withPos(costfun - (-trace(((C'*((C*(P0k\(C')))\C))/P0k)*(Pk*inv(P0k)))));
    
end
% keyboard;
costfun = costfun/100; % 10000;

pars = spot_sdp_default_options();
pars.verbose = 1;
% pars.scale_monomials = true;

sol = prog.minimize(costfun,@spot_mosek,pars);

% keyboard;


% sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

    disp('Backing off now...')

    % Back off on objective
    
    costfun_opt = double(sol.eval(costfun));
    prog = prog.withPos(costfun_opt + (options.backoff_percent/100)*abs(costfun_opt) - costfun);

    sol = prog.minimize(0,@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 1:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end

% for k = 1:9
%     Phi{k} = Phiold{k};
%     V0k = Vtraj0.getPoly(ts(k));
%     V{k} = V0k + x'*Phi{k}*x;
% end


end



function [V,rho,Phi] = findVRho(Vy,Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho0)

N = length(ts)-1;

disp('Step 2: Searching for V and rho...')

% V{N+1} = Vy{N+1};

% Phid{N+1} = zeros(length(x),length(x));


% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% Declare rho
[prog,rho] = prog.newPos(N+1);
prog = prog.withPos(rho(1) - rho0);

rhodot = msspoly('r',N);

% V at end
[prog,Phid{N+1}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{N+1} = mss_v2s(Phid{N+1});

% Normalization constraint
prog = prog.withEqs(trace(Phid{N+1}) - trace(Phiold{N+1}));

% Make V positive definite
[prog, Phislack] = prog.newPSD(length(x));
prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(N+1)) + Phid{N+1}));

% Initialize Phid{1}
Phid{1} = Phiold{1};

for k = N:-1:1
    if k > 1
        [prog,Phid{k}] = prog.newFree(nchoosek(length(x)+1,2));
        Phid{k} = mss_v2s(Phid{k});
        
        % Normalization constraint
        prog = prog.withEqs(trace(Phid{k}) - trace(Phiold{k}));
        
        
    end
    Phidotk = (Phid{k+1} - Phid{k})/(ts(k+1) - ts(k));
    V0k = Vtraj0.getPoly(ts(k));
    V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));
    
    % Compute V
    V{k} = V0k + x'*Phid{k}*x;
    
    % Compute Vdot
    Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phid{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;
    
    % Clean stuff
    Vdoty = clean(Vdoty,options.clean_tol);
    L1k = clean(L1{k},options.clean_tol);
    
    % Compute rhodot
    rhodot(k) = (rho(k+1)-rho(k))/(ts(k+1)-ts(k));
    
    % Declare SOS conditions
    prog = prog.withSOS(-Vdoty + rhodot(k) + L1k*(V{k}-rho(k)));
    % prog.sos = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));
    
    % Make V positive definite
    [prog,Phislack] = prog.newPSD(length(x));
    prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(k)) + Phid{k}));
    
    
    
end

% Solve SOS program
pars = spot_sdp_default_options();
pars.verbose = 1;
sol = prog.minimize(sum(rho),@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

%     disp('Backing off now...')
%
%     % Back off on objective
%     rhosum = double(sum(sol.eval(rho)));
%     prog = prog.withPos((-(1 - options.backoff_percent/100)*rhosum + sum(rho)));
%
%     sol = prog.minimize(0,@spot_mosek,pars);

%     if info.numerr == 2 || info.pinf == 1 || info.dinf == 1
%         keyboard;
%     end

rho = double(sol.eval(rho));

for k = 2:(N+1)
    Phi{k} = double(sol.eval(Phid{k}));
    V0k = Vtraj0.getPoly(ts(k));
    V{k} = V0k + x'*Phi{k}*x;
end
V{1} = Vtraj0.getPoly(ts(1));
Phi{1} = zeros(length(x),length(x));

end


function [Vy,rho,Phi] = rhoAndVLineSearch(Vtraj0,Vy,utraj,ts,forig_u,Phi,dts,options,u,ui,x,psys)

N = length(ts)-1;
disp('Step 0: Initializing rho and V with bisection search...')

% if (matlabpool('size')==0) matlabpool 4; end

rho = zeros(N+1,1);
rho(1) = options.rho0;

for k = 1:N
    rhonow = rho(k);
    rhomin = 0.5*rhonow;
    rhomax = 10*rhonow;
    rho(k+1) = fzero(@(rhonext) checkRho(Vtraj0,Vy,rhonext,utraj,ts,forig_u,Phi,dts,options,u,ui,x,k,rhonow,psys),[rhomin rhomax],optimset('TolX',1e-5));
    rho(k+1) = 1.01*rho(k+1); % To ensure feasibility
    
    if k > 0
        % Take this rho and find multipliers for it
        L1k = findLNext(Vtraj0,Vy{k},rho(k),rho(k+1),ts(k:k+1),forig_u{k},Phi{k},Phi{k+1},options,u,ui{k},x);
    
        % Take these multipliers and search for both V and rho
        [Vy{k+1},rho(k+1),Phi{k+1}] = findVRhoNext(Vtraj0,ts(k:k+1),forig_u{k},Phi(k:k+1),options,x,u,ui{k},L1k,rho(k),rho(k:k+1));
    
    %     Phi{k+2} = Phi{k+1};
    %     Vy{k+2} = Vtraj0.getPoly(ts(k+2)) + x'*Phi{k+2}*x;
    end
    
    rho
    
end

end

function L1f = findLNext(Vtraj0,Vy,rhonow,rhonext,ts,forig_u,Phinow,Phinext,options,u,ui,x)

disp('Searching for multipliers for next time step...')

prog = spotsosprog;
prog = prog.withIndeterminate(x);

tnow = ts(1);
tnext = ts(2);

rhodot = (rhonext - rhonow)/(tnext - tnow);


Phidotk = (Phinext - Phinow)/(tnext - tnow);
V0k = Vtraj0.getPoly(tnow);
V0dotk = Vtraj0.getPolyTimeDeriv(tnow);

% Compute Vdot
Vdoty = diff(V0k,x)*subss(forig_u,u,ui) + V0dotk + 2*x'*Phinow*subss(forig_u,u,ui) + x'*Phidotk*x;

% Clean stuff
V = clean(Vy,options.clean_tol);
Vdoty = clean(Vdoty,options.clean_tol);

% Declare multipliers
L1m = monomials(x,0:options.degL1);
[prog,l1] = prog.newFree(length(L1m));
% [prog,l1] = new(prog,length(L1m),'free');
L1 = l1'*L1m;

% Create gammas
[prog,gamma] = prog.newPos(1);
% [prog,gamma] = new(prog,1,'pos');

% Declare SOS conditions
prog = prog.withSOS(-gamma*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot + L1*(V-rhonow));
% prog.sos =  -gamma*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot(k) + L1*(V-rho(k));

% Solve SOS program
pars = spot_sdp_default_options();
pars.verbose = 1;
sol = prog.minimize(0,@spot_mosek,pars);

% Optimized multipliers
L1f = sol.eval(L1);

end

function [Vnext,rhonext,Phinext] = findVRhoNext(Vtraj0,ts,forig_u,Phiold,options,x,u,ui,L1,rho0,rhos)

disp('Searching for next V and rho...')

% sos program
prog = spotsosprog;
prog = prog.withIndeterminate(x);
% % Declare rho
% [prog,rho] = prog.newPos(2);
% prog = prog.withPos(rho(1) - rho0);

rho(1) = rhos(1);
rho(2) = rhos(2);


% Initialize Phid{1}
Phid{1} = Phiold{1};

% V at end
[prog,Phid{2}] = prog.newFree(nchoosek(length(x)+1,2));
Phid{2} = mss_v2s(Phid{2});

prog = prog.withPos(0.1*abs(Phiold{2}) - (Phiold{2} - Phid{2}));
prog = prog.withPos(0.1*abs(Phiold{2}) + (Phiold{2} - Phid{2}));

% % Normalization constraint
% prog = prog.withEqs(trace(Phid{2}) - trace(Phiold{2}));

% Make V positive definite
[prog, Phislack] = prog.newPSD(length(x));
prog = prog.withEqs(Phislack - (Vtraj0.S.eval(ts(2)) + Phid{2}));


Phidotk = (Phid{2} - Phid{1})/(ts(2) - ts(1));
V0k = Vtraj0.getPoly(ts(1));
V0dotk = Vtraj0.getPolyTimeDeriv(ts(1));

% Compute V
Vk = V0k + x'*Phid{1}*x;

% Compute Vdot
Vdoty = diff(V0k,x)*subss(forig_u,u,ui) + V0dotk + 2*x'*Phid{1}*subss(forig_u,u,ui) + x'*Phidotk*x;

% Clean stuff
Vdoty = clean(Vdoty,options.clean_tol);
L1k = clean(L1,options.clean_tol);

% Compute rhodot
rhodot = (rho(2)-rho(1))/(ts(2)-ts(1));

% Declare SOS conditions
prog = prog.withSOS(-Vdoty + rhodot + L1k*(Vk-rho(1)));
% prog.sos = -Vdoty + rhodot(k) + L1k*(V{k}-rho(k));


% % Solve SOS program
% pars = spot_sdp_default_options();
% pars.verbose = 1;
% sol = prog.minimize(sum(rho),@spot_mosek,pars);

costfun = 0;
for k = 2 % :N
    S0k = Vtraj0.S.eval(ts(k));
    % P0k = (S0k + Phiold{k})/rho(k); % P0k = 1000*eye(12);
    Pk = (S0k + Phid{k})/rho(k);
    
    [prog,tauk] = maxdet(prog,Pk); % tauk lower bounds det of Pk
    costfun = costfun - tauk;
    % prog = prog.withPos(costfun - (-tauk));
    
end

costfun = costfun/(10^2);

% Solve SOS program
pars = spot_sdp_default_options();
pars.verbose = 1;
% sol = prog.minimize(sum(rho),@spot_mosek,pars);
sol = prog.minimize(costfun,@spot_mosek,pars);




rhonext = double(sol.eval(rho(2)));
Phinext = double(sol.eval(Phid{2}));
Vnext = Vtraj0.getPoly(ts(2)) + x'*Phinext*x;

end




function rho = rhoLineSearch(Vtraj0,Vy,utraj,ts,forig_u,Phi,dts,options,u,ui,x,psys)

N = length(ts)-1;
disp('Step 0: Initializing rho with bisection search...')

% if (matlabpool('size')==0) matlabpool 4; end

rho = zeros(N+1,1);
rho(1) = options.rho0;

for k = 1:N
    rhonow = rho(k);
    rhomin = 0.5*rhonow;
    rhomax = 10*rhonow; % 10
    rho(k+1) = fzero(@(rhonext) checkRho(Vtraj0,Vy,rhonext,utraj,ts,forig_u,Phi,dts,options,u,ui,x,k,rhonow,psys),[rhomin rhomax],optimset('TolX',1e-5));
    rho(k+1) = 1.01*rho(k+1) % 1.001 To ensure feasibility
    
end

end

function gamma = checkRho(Vtraj0,Vy,rhonext,utraj,ts,forig_u,Phi,dts,options,u,ui,x,k,rho,psys)
% Compute rhodot
rhodot = (rhonext - rho)/dts(k);

prog = spotsosprog; % keyboard;
prog = prog.withIndeterminate(x);

Phidotk = (Phi{k+1} - Phi{k})/(ts(k+1) - ts(k));
V0k = Vtraj0.getPoly(ts(k));
V0dotk = Vtraj0.getPolyTimeDeriv(ts(k));

% Compute Vdot
% Vdoty = diff(V0k,x)*xdk + V0dotk + 2*x'*Phi{k}*xdk + x'*Phidotk*x;
Vdoty = diff(V0k,x)*subss(forig_u{k},u,ui{k}) + V0dotk + 2*x'*Phi{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;


% Vdoty = diff(V0k,x)*subs(psys.p_dynamics_traj.eval(ts(k)),u,zeros(4,1)) + V0dotk + 2*x'*Phi{k}*subss(forig_u{k},u,ui{k}) + x'*Phidotk*x;

% keyboard;

% Clean stuff
V = clean(Vy{k},options.clean_tol);
Vdoty = clean(Vdoty,options.clean_tol);

% Declare multipliers
L1m = monomials(x,0:options.degL1);
[prog,l1] = prog.newFree(length(L1m));
% [prog,l1] = new(prog,length(L1m),'free');
L1 = l1'*L1m;

% Create gammas
%  [prog,gamma] = prog.newPos(1);
[prog,gamma] = prog.newFree(1);

% Declare SOS conditions
prog = prog.withSOS(-gamma*(x'*x)^(deg(Vdoty,x)/2) - Vdoty + rhodot + L1*(V-rho));
% prog = prog.withSOS(-gamma - Vdoty + rhodot + L1*(V-rho));

% Solve SOS program
pars = spot_sdp_default_options();
pars.verbose = 1;
% keyboard;
sol = prog.minimize(-gamma,@spot_mosek,pars);

if strcmp(sol.info.solverInfo.itr.prosta,'PRIMAL_INFEASIBLE')
    gamma = -1.0;
else
    
    gamma = double(sol.eval(gamma));
    
    if strcmp(sol.info.solverInfo.itr.prosta,'UNKNOWN')
        gamma = -1.0;
    end
    
end


end





%% Helper/Debugging functions after this%%




function [mi,ma]=plotPoly(x,P,rho)
if(nargin<3) rho=0; end
[X1,X2]=ndgrid(-2:.1:2,-2:.1:2);
Ps=reshape(doubleSafe(msubs(P,x,[X1(:)';X2(:)'])),size(X1));
mi=min(min(Ps));
ma=max(max(Ps));
surf(X1,X2,Ps); colorbar;
view(0,90);
hold on;
[c,h]=contour3(X1,X2,Ps,[rho,rho]);
set(h,'EdgeColor',[1 1 1],'LineWidth',4);
end

function [Vmin,b] = minimumV(x,V)
if (deg(V,x)>2)
    prog = mssprog;
    [prog,slack] = new(prog,1,'free');
    prog.sos = slack + V;
    [prog,info] = sedumi(prog,slack,0);
    Vmin = -doubleSafe(prog(slack));
else
    H = doubleSafe(0.5*diff(diff(V,x)',x));
    b = -0.5*(H\doubleSafe(subs(diff(V,x),x,0*x)'));
    Vmin = subs(V,x,b);
end
end

function m=sampleCheck(x,V,Vdot,rho,rhodot)
if (deg(V,x)>2) error('only checks quadratics'); end

n=length(x);
K=100;
X = randn(n,K);
X = X./repmat(sqrt(sum(X.^2,1)),n,1);

H = doubleSafe(0.5*diff(diff(V,x)',x));
b = -0.5*(H\doubleSafe(subs(diff(V,x),x,0*x)'));

try
    X = repmat(b,1,K) + (H/(doubleSafe(rho-subs(V,x,b))+eps))^(-1/2)*X;
catch
    keyboard;
end
m=max(doubleSafe(msubs(Vdot,x,X))) - rhodot;
if (m>0)
    warning('found a positive Vdot');
end
end


function y=doubleSafe(x)
y=double(x);
if (~isa(y,'double')) error('double failed'); end
end

function tf=equalpoly(A,B)

x=decomp(A);
sizecheck(A,1); sizecheck(B,1);
if (deg(A,x)>2 || deg(B,x)>2) error('not supported yet'); end  % but not very hard!

C=A-B;
if (any(abs(doubleSafe(subs(C,x,0*x)))>1e-4))
    tf=false; return;
end

if (any(abs(doubleSafe(subs(diff(C,x),x,0*x)))>1e-4))
    tf=false; return;
end

if (any(abs(doubleSafe(subs(diff(diff(C,x)',x),x,0*x)))>1e-4))
    tf=false; return;
end

tf=true;
end