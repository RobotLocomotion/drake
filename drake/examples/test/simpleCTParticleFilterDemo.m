function simpleCTParticleFilterDemo

% some options/parameters
plot_histogram = true;
time_reversed_system = false;
noise_covariance = .1;  % pretty robust stationary distribution
noise_covariance = 1.2;   % should see some "escape attempts"
%noise_covariance = 2;   % see definite particle loss nominal system


tmppath = addpathTemporary(fullfile(getDrakePath,'examples'));

if (time_reversed_system)
  % the time-reversed system is interesting, too
  sys = FunctionHandleSystem(1,0,0,1,false,true,@(t,x,u) x-x.^3,[],@(t,x,u)x);
else
  sys = SimpleCTExample;
end

sys = DrakeSystemWGaussianNoise(sys,noise_covariance,[],0,.1);

tspan = 0:.1:30;

if (plot_histogram)
  num_particles = 100000;
  figure(1); clf; hold on;  xs = -1.5:.1:1.5; plot(xs,2500*dynamics(sys,0,xs,[]),'Color','k','LineWidth',4); % for histogram
else % plot time
  num_particles = 50;
  figure(1); clf; hold on; axis([tspan(1) tspan(end) -2 2]);
end

particleFilterDemo(sys,randn(1,num_particles),tspan,@draw_function);

function draw_function(t,x)

if (plot_histogram)
  h = histogram(x,[-1.5:.05:1.5]); axis([-1.5 1.5 -5000 15000]);
  set(h,'FaceColor','r');
  drawnow;
  delete(h)
else % plot time
  plot(t,x,'b.','MarkerSize',20)
  drawnow;
end

end

end