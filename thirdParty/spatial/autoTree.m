function  model = autoTree( nb, bf, skew, taper )

% autoTree  Create System Models of Kinematic Trees
% autoTree(nb,bf,skew,taper) creates system models of kinematic trees
% having revolute joints.  nb and bf specify the number of bodies in the
% tree, and the branching factor, respectively.  The latter is the average
% number of children of a nonterminal node, and must be >=1.  bf=1 produces
% an unbranched tree; bf=2 produces a binary tree; and non-integer values
% produce trees in which the number of children alternates between
% floor(bf) and ceil(bf) in such a way that the average is bf.  Trees are
% constructed (and numbered) breadth-first.  Link i is a thin-walled
% cylindrical tube of length l(i), radius l(i)/20, and mass m(i), lying
% between 0 and l(i) on the x axis of its local coordinate system.  The
% values of l(i) and m(i) are determined by the tapering coefficient:
% l(i)=taper^(i-1) and m(i)=taper^(3*(i-1)).  Thus, if taper=1 then
% m(i)=l(i)=1 for all i.  The inboard joint axis of link i lies on the
% local z axis, and its outboard axis passes through the point (l(i),0,0)
% and is rotated about the x axis by an angle of skew radians relative to
% the inboard axis.  If the link has more than one outboard joint then they
% all have the same axis.  If skew=0 then the mechanism is planar.  The
% final one, two or three arguments can be omitted, in which case they
% assume default values of taper=1, skew=0 and bf=1.

if nargin < 4
  taper = 1;
end
if nargin < 3
  skew = 0;
end
if nargin < 2
  bf = 1;
end

model.NB = nb;
model.pitch = zeros(1,nb);

for i = 1:nb
  model.parent(i) = floor((i-2+ceil(bf))/bf);
  if model.parent(i) == 0
    model.Xtree{1} = Xtrans([0 0 0]);
  else
    model.Xtree{i} = Xrotx(skew) * Xtrans([len(model.parent(i)),0,0]);
  end
  len(i) = taper^(i-1);
  mass = taper^(3*(i-1));
  CoM = len(i) * [0.5,0,0];
  Icm = mass * len(i)^2 * diag([0.0025,1.015/12,1.015/12]);
  model.I{i} = mcI( mass, CoM, Icm );
end

% drawing instructions

model.appearance{1} = {...
  { 'box', [-0.2, 0.2; -0.3, 0.3; -0.2, -0.06] } };

p0 = -1;
for i = 1:nb
  p1 = model.parent(i);
  tap = taper^(i-1);
  if p1 == 0
    ptap = 1;
  else
    ptap = taper^(p1-1);
  end
  if ( p1 > p0 )
    model.appearance{i+1} = {...
      { 'cyl', [0.5 0 0]*tap, 0.05*tap, 1*tap, 'X' },...
      { 'cyl', [0 0 0], 0.08*ptap, 0.12*ptap, 'Z' } };
    p0 = p1;
  else
    model.appearance{i+1} = {...
      { 'cyl', [0.5 0 0]*tap, 0.05*tap, 1*tap, 'X' } };
  end
end
