function  qdd = FDabp( model, q, qd, tau, f_ext, grav_accn )

% FDabp  Forward Dynamics (planar) via Articulated-Body Algorithm.
% FDabp(model,q,qd,tau,f_ext,grav_accn) calculates the forward dynamics of
% a kinematic tree via the articulated-body algorithm, evaluated using
% planar vectors.  q, qd and tau are vectors of joint position, velocity
% and force variables; and the return value is a vector of joint
% acceleration variables.  f_ext is a cell array specifying external forces
% acting on the bodies.  If f_ext == {} then there are no external forces;
% otherwise, f_ext{i} is a planar force vector giving the force acting on
% body i, expressed in body i coordinates.  Empty cells in f_ext are
% interpreted as zero forces.  grav_accn is a 2D vector expressing the
% linear acceleration due to gravity in the x-y plane.  The arguments f_ext
% and grav_accn are optional, and default to zero (i.e., {} and [0 0],
% respectively) if omitted.

if nargin < 6
  a_grav = [0;0;0];
else
  a_grav = [0;grav_accn(1);grav_accn(2)];
end

external_force = ( nargin > 4 && length(f_ext) > 0 );

for i = 1:model.NB
  [ XJ, S{i} ] = jcalcp( model.jcode(i), q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    c{i} = zeros(3,1);
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    c{i} = crmp(v{i}) * vJ;
  end
  IA{i} = model.I{i};
  pA{i} = crfp(v{i}) * model.I{i} * v{i};
  if external_force && length(f_ext{i}) > 0
    pA{i} = pA{i} - f_ext{i};
  end
end

for i = model.NB:-1:1
  U{i} = IA{i} * S{i};
  d{i} = S{i}' * U{i};
  u{i} = tau(i) - S{i}'*pA{i};
  if model.parent(i) ~= 0
    Ia = IA{i} - U{i}/d{i}*U{i}';
    pa = pA{i} + Ia*c{i} + U{i} * u{i}/d{i};
    IA{model.parent(i)} = IA{model.parent(i)} + Xup{i}' * Ia * Xup{i};
    pA{model.parent(i)} = pA{model.parent(i)} + Xup{i}' * pa;
  end
end

for i = 1:model.NB
  if model.parent(i) == 0
    a{i} = Xup{i} * -a_grav + c{i};
  else
    a{i} = Xup{i} * a{model.parent(i)} + c{i};
  end
  qdd(i,1) = (u{i} - U{i}'*a{i})/d{i};
  a{i} = a{i} + S{i}*qdd(i);
end
