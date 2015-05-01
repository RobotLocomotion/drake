classdef LinearGaussianExample < StochasticDrakeSystem

 methods
   function obj = LinearGaussianExample
     obj = obj@StochasticDrakeSystem(...
       1, ... % number of continuous states
       0, ... % number of discrete states
       0, ... % number of inputs
       1, ... % number of outputs
       false, ...  % not direct feedthrough
       true, ...   % time invariant
       1, ... % number of noise inputs
       .01);  % time constant of w(t)
   end

   function xcdot = stochasticDynamics(obj,t,x,u,w)
     xcdot = -x + w;
   end

   function x0 = getInitialState(obj)
     x0 = randn;
   end

   function y = stochasticOutput(obj,t,x,u,w);
     y=x;
   end
 end

 methods (Static)
   function run
     sys=LinearGaussianExample;
     fnplt(simulate(sys,[0 1]));
%     S=.5; tspan=[0 1]; x0=0;
%     options.rho0 = 1;
%     stochasticFiniteTimeVerification(sys,S,tspan,x0,options);
%     prob=monteCarlo(sys,S,tspan,x0)
   end
 end

 methods
   function prob=monteCarlo(obj,S,tspan,x0)
     N=200;

     ok=checkDependency('distcomp');  % initialize toolbox if it exists
     
     parfor i=1:N
       xtraj = simulate(obj,tspan,x0);
       xs=eval(xtraj,xtraj.getBreaks());
       m{i}=max(sum((S*xs).*xs,1));
       xend{i}=xs(:,end);
     end
     prob = sum([m{:}]>1)/length(m);

% for comparison with kalman
%     figure(12);
%     hist([xend{:}],linspace(-5,5,41)); axis([-5,5,0,N]);
   end

   function plotKalman(obj,tspan)
     % for comparison w/ monte-carlo, plot the distribution that I would
     % expect to get at the end of tspan (with x0=0) given the kalman
     % filter equations.
     % (using symbols from Kalman-Bucy subsection of http://en.wikipedia.org/wiki/Kalman_filter)

     F=-1; Q=1;
     %H=1; R=0; are unused (no observations)

     m0 = 0;  P0 = 0;  % initial mean and covariance
     Pdot = @(t,P)(F*P+P*F''+Q);
     P=matrixODE(@ode45,Pdot,tspan,P0);  %note: this is overkill, since P is a scalar in this case
     Pf = P.eval(tspan(end));

     pdf = @(xs)(1/sqrt(2*pi*Pf^2)*exp(-xs.^2/(2*Pf^2)));
     figure(13)
     xs=-5:.01:5;
     plot(xs,200*pdf(xs));
   end
 end
end
