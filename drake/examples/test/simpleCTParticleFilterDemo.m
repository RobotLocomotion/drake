function simpleCTParticleFilterDemo

tmppath = addpathTemporary(fullfile(getDrakePath,'examples'));

noise_covariance = .1;  % pretty robust stationary distribution
%noise_covariance = 2;   % should see some "escapes"

sys = DrakeSystemWGaussianNoise(SimpleCTExample,noise_covariance,[],0,.1);

tspan = 0:.1:30;

%figure(1); clf; hold on; axis([tspan(1) tspan(end) -2 2]);
figure(1); clf; hold on;  xs = -1.5:.1:1.5; plot(xs,2500*dynamics(sys,0,xs,[]),'Color','k','LineWidth',4); % for histogram

particleFilterDemo(sys,randn(1,100000),tspan,@draw_function);

end

function draw_function(t,x)

% plot(t,x,'b.','MarkerSize',20)
h = histogram(x,[-1.5:.05:1.5]); axis([-1.5 1.5 -5000 15000]);
set(h,'FaceColor','r');
drawnow;
delete(h)

end