function  model = autoTreep( nb, bf, taper )

% autoTreep  System Models of Kinematic Trees, using Planar Vectors
% autoTreep(nb,bf,taper) creates planar-vector system models of planar
% kinematic trees having revolute joints.  autoTreep(nb,bf,taper) is the
% planar-vector equivalent of autoTree(nb,bf,0,taper).  nb and bf specify
% the number of bodies in the tree, and the branching factor, respectively.
% The latter is the average number of children of a nonterminal node, and
% must be >=1.  bf=1 produces an unbranched tree; bf=2 produces a binary
% tree; and non-integer values of bf produce trees in which the number of
% children alternates between floor(bf) and ceil(bf) in such a way that the
% average is bf.  Trees are constructed (and numbered) breadth-first.  For
% the purpose of calculating geometric and inertia parameters, link i is
% regarded as a thin-walled cylindrical tube of length l(i), radius
% l(i)/20, and mass m(i), lying between 0 and l(i) on the x axis of its
% local coordinate system.  The values of l(i) and m(i) are determined by
% the tapering coefficient: l(i)=taper^(i-1) and m(i)=taper^(3*(i-1)).
% Thus, if taper=1 then m(i)=l(i)=1 for all i.  The inboard joint axis of
% link i rotates about the point (0,0), and its outboard joint (or joints)
% rotate about (l(i),0).  The final one or two arguments can be omitted, in
% which case they assume default values of taper=1 and bf=1.

if nargin < 3
  taper = 1;
end
if nargin < 2
  bf = 1;
end

model.NB = nb;
model.jcode = ones(1,nb);		% every joint is revolute

for i = 1:nb
  model.parent(i) = floor((i-2+ceil(bf))/bf);
  if model.parent(i) == 0
    model.Xtree{1} = Xpln(0,[0 0]);
  else
    model.Xtree{i} = Xpln(0,[taper^(model.parent(i)-1),0]);
  end
  len = taper^(i-1);
  mass = taper^(3*(i-1));
  model.I{i} = mcIp( mass, [len/2,0], mass*len^2*1.015/12 );
end
