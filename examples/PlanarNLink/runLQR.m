function runLQR(N)

if (nargin<1) N=max(ceil(8*rand),2); end

p = PlanarNLink(N);
v = p.constructVisualizer();
v.axis=.5*[-1 1 -1 1]+N*1.5*[-1 1 -1 1];

Q = blkdiag(10*eye(N),eye(N)); R = eye(N-1);
x0 = Point(p.getStateFrame,[pi;zeros(2*N-1,1)]);
[c,V] = tilqr(p,x0,Point(p.getInputFrame,zeros(N-1,1)),Q,R);

sys=feedback(p,c);

% disp('taking taylor approx..'); drawnow;
% pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
% 
% disp('computing ROA...'); drawnow;
% options=struct();
% %options.method='bilinear';
% V=regionOfAttraction(pp,x0,[],options)
% 
% disp('simulating.');

for i=1:5;
  xtraj = sys.simulate([0 2],double(x0)+.01*randn(2*N,1));
  v.playback(xtraj);
end

disp('done.');
