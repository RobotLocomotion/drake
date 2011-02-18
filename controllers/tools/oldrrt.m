function [xtraj,utraj] = rrt(sys,DISTFUN,x0,xG,xRange,bWrap,ubins,options)

% RRT implementation
% 
%  x0 is initial condition
%  xG is goal, use [] for coverage RRT
%  xRange is an nX by 2 matrix with limits
%  bWrap is an nX by 1 boolean
%  ubins contains discrete actions to try
%  options:
%    dt (default .1)
%    numNodes (default 10000)
%

if (nargin<6) options = struct(); end

% default options:
if (~isfield(options,'dt')) options.dt = 0.1; end
if (~isfield(options,'numNodes')) options.numNodes = 10000; end
if (~isfield(options,'iPlotInterval')) options.iPlotInterval = 100; end
if (~isfield(options,'goalBias')) options.goalBias = 0.05; end
if (~isfield(options,'converged')) options.converged = .1; end
% to add : plot dims

t_tape = [];
x_tape = [];
u_tape = [];

N = options.numNodes;  % for pre-allocating memory
G = repmat(x0,1,N);  % list of nodes
P = ones(1,N);       % list of parents (indexes into G)
U = ones(1,N);       % action required to transition from G(P(i),:) to G(i,:)

nX = length(x0);
nU = length(ubins);
Ulist = ubins{1}(:)';  if (length(ubins)>1) error('multiple actions not implemented yet'); end

clf
% for now, just plot the first two dimensions:
axis([xRange(1,1),xRange(1,2),xRange(2,1),xRange(2,2)]);  hold on;
plot(x0(1),x0(2),'gx',xG(1),xG(2),'rx','MarkerSize',20,'LineWidth',3);

for n=2:N
  if (~isempty(xG) && rand<options.goalBias)
    xsample = xG;
  else
    xsample = rand(nX,1).*(xRange(:,2)-xRange(:,1)) + xRange(:,1);
  end
  h = plot(xsample(1),xsample(2),'r.');
  d = DISTFUN(xsample,G(:,1:n-1));
  [y,i] = min(d);

  h = [h,line([G(1,i),xsample(1)],[G(2,i),xsample(2)],'Color',[0 0 1])];
  h = [h,plot(G(1,i),G(2,i),'b.')];

  % try expanding many actions from the closest point, and keep the best
  for ui=1:length(Ulist)
    %      [t,x] = ode45(@(t,x) dynamics(t,x,Ulist(ui)),[dt 0],G(:,i));
    %      xn_c(:,ui) = x(end,:)';
    xn_c(:,ui) = G(:,i) + options.dt*sys.dynamics(0,G(:,i),Ulist(ui));
    h=[h,line([G(1,i),xn_c(1,ui)],[G(2,i),xn_c(2,ui)],'Color',[0 1 0])];
  end
  d = DISTFUN(xsample,xn_c);
  [y,ui] = min(d);
  xn = xn_c(:,ui);
  U(1,n) = Ulist(ui);
  [xn,bwrapped] = wrap(xRange,bWrap,xn);
    
  if (~bwrapped)
    line([G(1,i),xn(1)],[G(2,i),xn(2)],'Color',0.3*[1 1 1]);
  end
  h=[h,line([G(1,i),xn(1)],[G(2,i),xn(2)],'Color',[1 0 0])];
  G(:,n) = xn;
  P(1,n) = i;
  
  if (~isempty(xG))
    d = norm(xG-xn);
    if (d < options.converged)
      xbi = n;
      while (xbi(1) ~= 1)
        line([G(1,xbi),G(1,P(xbi))],[G(2,xbi),G(2,P(xbi))],'Color',[1 0 0],'LineWidth',2);
        u_tape = [U(:,xbi),u_tape];
        x_tape = [G(:,xbi),x_tape];
        xbi = P(xbi);
      end
      t_tape = options.dt*(0:(size(u_tape,2)-1))';

      % for compatability w/ ode, etc
      x_tape = x_tape';
      u_tape = u_tape';
      return;
    end
  end
    
  if (mod(n,options.iPlotInterval)==0)
    drawnow;
  end
%  pause;
  delete(h);
  
end


function [xw,bwrapped] = wrap(xRange,bWrap,x)

bwrapped = zeros(1,size(x,2));
for i=1:size(x,1)
  if (bWrap(i)) 
    xw(i,:) = mod(x(i,:)-xRange(i,1),xRange(i,2)-xRange(i,1))+xRange(i,1);
    bwrapped = bwrapped | abs(xw(i,:)-x(i,:))>1e-5;
  else
    xw(i,:) = x(i,:);
  end
end
