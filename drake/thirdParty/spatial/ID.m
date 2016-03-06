function  tau = ID( model, q, qd, qdd, f_ext, grav_accn )

% ID  Inverse Dynamics via Recursive Newton-Euler Algorithm
% ID(model,q,qd,qdd,f_ext,grav_accn) calculates the inverse dynamics of a
% kinematic tree via the recursive Newton-Euler algorithm.  q, qd and qdd
% are vectors of joint position, velocity and acceleration variables; and
% the return value is a vector of joint force variables.  f_ext is a cell
% array specifying external forces acting on the bodies.  If f_ext == {}
% then there are no external forces; otherwise, f_ext{i} is a spatial force
% vector giving the force acting on body i, expressed in body i
% coordinates.  Empty cells in f_ext are interpreted as zero forces.
% grav_accn is a 3D vector expressing the linear acceleration due to
% gravity.  The arguments f_ext and grav_accn are optional, and default to
% the values {} and [0,0,-9.81], respectively, if omitted.

if nargin < 6
  a_grav = [0;0;0;0;0;-9.81];
else
  a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

external_force = ( nargin > 4 && length(f_ext) > 0 );

for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.pitch(i), q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    a{i} = Xup{i} * -a_grav + S{i}*qdd(i);
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    a{i} = Xup{i}*a{model.parent(i)} + S{i}*qdd(i) + crm(v{i})*vJ;
  end
  f{i} = model.I{i}*a{i} + crf(v{i})*model.I{i}*v{i};
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
