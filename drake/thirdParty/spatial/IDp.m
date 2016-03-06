function  tau = IDp( model, q, qd, qdd, f_ext, grav_accn )

% IDp  Inverse Dynamics (planar) via Recursive Newton-Euler Algorithm.
% IDp(model,q,qd,qdd,f_ext,grav_accn) calculates the inverse dynamics of a
% kinematic tree via the recursive Newton-Euler algorithm, evaluated using
% planar vectors.  q, qd and qdd are vectors of joint position, velocity
% and acceleration variables; and the return value is a vector of joint
% force variables.  f_ext is a cell array specifying external forces acting
% on the bodies.  If f_ext == {} then there are no external forces;
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
    a{i} = -Xup{i}*a_grav + S{i}*qdd(i);
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    a{i} = Xup{i}*a{model.parent(i)} + S{i}*qdd(i) + crmp(v{i})*vJ;
  end
  f{i} = model.I{i}*a{i} + crfp(v{i})*model.I{i}*v{i};
  if external_force && length(f_ext{i}) > 0
    f{i} = f{i} - f_ext{i};
  end
end

for i = model.NB:-1:1
  tau(i,1) = S{i}' * f{i};
  if model.parent(i) ~= 0
    f{model.parent(i)} = f{model.parent(i)} + Xup{i}'*f{i};
  end
end
