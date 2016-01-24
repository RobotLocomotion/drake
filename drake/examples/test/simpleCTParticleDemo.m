function simpleCTParticleDemo(options)

% some options/parameters
if nargin<1, options = struct(); end
if ~isfield(options,'plot_histogram'), options.plot_histogram = true; end
if ~isfield(options,'time_reversed_system'), options.time_reversed_system = true; end

if (options.plot_histogram && verLessThan('matlab','8.4'))
  warning('the histogram method I use here was only introduced in 2014b')
  options.plot_histogram = false;
end

noise_covariance = .1;  % pretty robust stationary distribution
%noise_covariance = 1.2;   % should see some "escape attempts"
%noise_covariance = 2;   % see definite particle loss nominal system
if isfield(options,'noise_covariance'), noise_covariance = options.noise_covariance; end

tmppath = addpathTemporary(fullfile(getDrakePath,'examples'));

if (options.time_reversed_system)
  % the time-reversed system is interesting, too
  sys = FunctionHandleSystem(1,0,0,1,false,true,@(t,x,u) x-x.^3,[],@(t,x,u)x);
else
  sys = SimpleCTExample;
end

sys = DrakeSystemWGaussianNoise(sys,noise_covariance,[],0,.1);

tspan = 0:.1:30;

if (options.plot_histogram)
  num_particles = 100000;
  figure(1); clf; hold on;  xs = -1.5:.1:1.5; plot(xs,dynamics(sys,0,xs,[]),'Color','k','LineWidth',4); % for histogram
else % plot time
  num_particles = 50;
  figure(1); clf; hold on; axis([tspan(1) tspan(end) -2 2]);
end

particleDemo(sys,randn(1,num_particles),tspan,@draw_function);

function draw_function(t,x)

if (options.plot_histogram)
  h = histogram(x,[-1.5:.05:1.5],'Normalization','pdf'); axis([-1.5 1.5 -2 3.5]);
  set(h,'FaceColor','r');
  xlabel('x');
  ylabel('p(x)');
  title(['t=',num2str(t)]);
  drawnow;
  delete(h)
else % plot time
  plot(t,x,'b.','MarkerSize',20)
  xlabel('t');
  ylabel('x');
  drawnow;
end

end

end