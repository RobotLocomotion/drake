function  [H,C] = HandCp( model, q, qd, f_ext, grav_accn, jsign )

% HandCp  Coefficients of equation of motion (planar vectors).
% [H,C]=HandCp(model,q,qd,f_ext,grav_accn) calculates the coefficients of
% the joint-space equation of motion, tau=H(q)*qdd+C(q,qd,f_ext), where q,
% qd and qdd are the joint position, velocity and acceleration vectors, H
% is the joint-space inertia matrix, C is the vector of gravity,
% external-force and velocity-product terms, and tau is the joint force
% vector.  Algorithm: recursive Newton-Euler for C, and
% Composite-Rigid-Body for H, both evaluated using planar vectors.  f_ext
% is a cell array specifying external forces acting on the bodies.  If
% f_ext == {} then there are no external forces; otherwise, f_ext{i} is a
% planar force vector giving the force acting on body i, expressed in body
% i coordinates.  Empty cells in f_ext are interpreted as zero forces.
% grav_accn is a 2D vector expressing the linear acceleration due to
% gravity in the x-y plane.  The arguments f_ext and grav_accn are
% optional, and default to zero (i.e., {} and [0 0], respectively) if
% omitted.
%
% UPDATED by russt:  f_ext is an empty or (sparse) 6 x model.NB matrix

if nargin < 5
  a_grav = [0;0;0];
else
  a_grav = [0;grav_accn(1);grav_accn(2)];
end

external_force = ( nargin > 3 && length(f_ext) > 0 );

for i = 1:model.NB
  [ XJ, S{i} ] = jcalcp( model.jcode(i), q(i), model.jsign(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    avp{i} = Xup{i} * -a_grav;
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    avp{i} = Xup{i}*avp{model.parent(i)} + crmp(v{i})*vJ;
  end
  fvp{i} = model.I{i}*avp{i} + crfp(v{i})*model.I{i}*v{i};
  if external_force
    fvp{i} = fvp{i} - f_ext(:,i);
  end
end

IC = model.I;				% composite inertia calculation

C = zeros(model.NB,1)*q(1);

for i = model.NB:-1:1
  C(i,1) = S{i}' * fvp{i};
  if model.parent(i) ~= 0
    fvp{model.parent(i)} = fvp{model.parent(i)} + Xup{i}'*fvp{i};
    IC{model.parent(i)} = IC{model.parent(i)} + Xup{i}'*IC{i}*Xup{i};
  end
end

% minor adjustment to make TaylorVar work better.
%H = zeros(model.NB);
H=zeros(model.NB)*q(1);

for i = 1:model.NB
  fh = IC{i} * S{i};
  H(i,i) = S{i}' * fh;
  j = i;
  while model.parent(j) > 0
    fh = Xup{j}' * fh;
    j = model.parent(j);
    H(i,j) = S{j}' * fh;
    H(j,i) = H(i,j);
  end
end
