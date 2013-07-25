function runPassive(N)

if (nargin<1) N = ceil(10*rand); end

sys = PlanarNLink(N);
x=sys.simulate([0 5],randn(2*N,1));
v=sys.constructVisualizer();
v.axis=[-.5 .5 -1.2 1.2]+(N-1)*[-1 1 -1.2 1.2];
v.playback(x);
