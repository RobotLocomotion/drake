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
% NOTE: Heavily modified by russt
%   - f_ext is an empty or (sparse) 6 x model.NB matrix
%   - adds support for floating joints
%   - adds support for TaylorVars

if nargin < 5
  a_grav = [0;0;0;0;0;-9.81];
else
  a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

external_force = ( nargin > 3 && length(f_ext) > 0 );

n=0;
for i = 1:model.NB
  if model.floatingbase(i)==1
    XJ = Xtrans(q(n+(1:3)));
    vJ = [zeros(3,1);qd(n+(1:3))];  % spatial vel is [omega;v]
    S{n+1} = [0;0;0;1;0;0];
    S{n+2} = [0;0;0;0;1;0];
    S{n+3} = [0;0;0;0;0;1];
    
    XJ = Xrotz(q(n+6))*XJ;
    S{n+4} = [0;0;1;0;0;0];
    vJ = Xrot(q(n+6))*vJ + S{n+4}*qd(n+4);
    
    XJ = Xroty(q(n+5))*XJ; 
    S{n+5} = [0;1;0;0;0;0];
    S{n+6} = [1;0;0;0;0;0];
    vJ = Xrotz(q(n+6))*vJ + [0;0;1;0;0;0]*qd(n+;
    Xrotx(q(n+4))*Xroty(q(n+5))*Xrotz(q(n+6))
    n=n+6;
  elseif model.floatingbase(i)==2
    n=n+7;
    error('dynamics for quaternion floating bases are not implemented yet')
  else % it's just a normal joint
    n=n+1;
    [ XJ, S{n} ] = jcalc( model.pitch(i), q(n) );
    vJ = S{n}*qd(n);
  end
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

C = zeros(n,1)*q(1);

for i = model.NB:-1:1
  if model.floatingbase(i)==1
    error('todo: fill out C');
    n=n-6;
  elseif model.floatingbase(i)==2
    error('todo: fill out C');
    n=n-6;
  else
    C(n,1) = S{i}' * fvp{i};
    n=n-1;
  end
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
