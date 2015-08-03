function  [H,C] = HandC( model, q, qd, f_ext, grav_accn )

% HandC  Calculate coefficients of equation of motion.
% [H,C]=HandC(model,q,qd,f_ext,grav_accn) calculates the coefficients of
% the joint-space equation of motion, tau=H(q)qdd+C(d,qd,f_ext), where q,
% qd and qdd are the joint position, velocity and acceleration vectors, H
% is the joint-space inertia matrix, C is the vector of gravity,
% external-force and velocity-product terms, and tau is the joint force
% vector.  Algorithm: recursive Newton-Euler for C, and
% Composite-Rigid-Body for H.  f_ext is a cell array specifying external
% forces acting on the bodies.  If f_ext == {} then there are no external
% forces; otherwise, f_ext{i} is a spatial force vector giving the force
% acting on body i, expressed in body i coordinates.  Empty cells in f_ext
% are interpreted as zero forces.  grav_accn is a 3D vector expressing the
% linear acceleration due to gravity.  The arguments f_ext and grav_accn
% are optional, and default to the values {} and [0,0,-9.81], respectively,
% if omitted.
%
% UPDATED by russt:  f_ext is an empty or (sparse) 6 x model.NB matrix


if nargin < 5
  a_grav = [0;0;0;0;0;-9.81];
else
  a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

external_force = ( nargin > 3 && length(f_ext) > 0 );

for i = 1:model.NB
  n = model.position_num(i);
  [ XJ, S{i} ] = jcalc( model.pitch(i), q(n) );
  vJ = S{i}*qd(n);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    avp{i} = Xup{i} * -a_grav;
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    avp{i} = Xup{i}*avp{model.parent(i)} + crm(v{i})*vJ;
  end
  fvp{i} = model.I{i}*avp{i} + crf(v{i})*model.I{i}*v{i};
  if external_force
    fvp{i} = fvp{i} - f_ext(:,i);
  end
end

IC = model.I;				% composite inertia calculation

C = zeros(model.NB,1)*q(1);

for i = model.NB:-1:1
  n = model.position_num(i);
  C(n,1) = S{i}' * fvp{i};
  if model.parent(i) ~= 0
    fvp{model.parent(i)} = fvp{model.parent(i)} + Xup{i}'*fvp{i};
    IC{model.parent(i)} = IC{model.parent(i)} + Xup{i}'*IC{i}*Xup{i};
  end
end

% minor adjustment to make TaylorVar work better.
%H = zeros(model.NB);
H=zeros(model.NB)*q(1);

for i = 1:model.NB
  n = model.position_num(i);
  fh = IC{i} * S{i};
  H(n,n) = S{i}' * fh;
  j = i;
  while model.parent(j) > 0
    fh = Xup{j}' * fh;
    j = model.parent(j);
    np = model.position_num(j);
    H(n,np) = S{j}' * fh;
    H(np,n) = H(n,np);
  end
end
