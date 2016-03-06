function  [qdd_out,tau_out] = HD( model, fd, q, qd, qdd, tau, f_ext, grav_accn)

% HD  Articulated-Body Hybrid Dynamics Algorithm
% [qdd_out,tau_out]=HD(model,fd,q,qd,qdd,tau,f_ext,grav_accn) calculates
% the hybrid dynamics of a kinematic tree using the articulated-body
% algorithm.  fd is an array of boolean values such that fd(i)==1 if joint
% i is a forward-dynamics joint, and fd(i)==0 otherwise.  If fd(i)==1 then
% tau(i) contains the given force at joint i, and the value of qdd(i) is
% ignored; and if fd(i)==0 then qdd(i) contains the given acceleration at
% joint i, and the value of tau(i) is ignored.  Likewise, if fd(i)==1 then
% qdd_out(i) contains the calculated acceleration at joint i, and
% tau_out(i) contains the given force copied from tau(i); and if fd(i)==0
% then tau_out(i) contains the calculated force and qdd_out(i) the given
% acceleration copied from qdd(i).  Thus, the two output vectors are always
% fully instantiated.  f_ext is a cell array specifying external forces
% acting on the bodies.  If f_ext == {} then there are no external forces;
% otherwise, f_ext{i} is a spatial force vector giving the force acting on
% body i, expressed in body i coordinates.  Empty cells in f_ext are
% interpreted as zero forces.  grav_accn is a 3D vector expressing the
% linear acceleration due to gravity.  The arguments f_ext and grav_accn
% are optional, and default to the values {} and [0,0,-9.81], respectively,
% if omitted.

if nargin < 8
  a_grav = [0;0;0;0;0;-9.81];
else
  a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

external_force = ( nargin > 6 && length(f_ext) > 0 );

for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.pitch(i), q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    c{i} = zeros(6,1);
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    c{i} = crm(v{i}) * vJ;
  end
  if fd(i) == 0
    c{i} = c{i} + S{i} * qdd(i);
  end
  IA{i} = model.I{i};
  pA{i} = crf(v{i}) * model.I{i} * v{i};
  if external_force && length(f_ext{i}) > 0
    pA{i} = pA{i} - f_ext{i};
  end
end

for i = model.NB:-1:1
  if fd(i) == 0
    if model.parent(i) ~= 0
      Ia = IA{i};
      pa = pA{i} + IA{i}*c{i};
      IA{model.parent(i)} = IA{model.parent(i)} + Xup{i}' * Ia * Xup{i};
      pA{model.parent(i)} = pA{model.parent(i)} + Xup{i}' * pa;
    end
  else
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
end

for i = 1:model.NB
  if model.parent(i) == 0
    a{i} = Xup{i} * -a_grav + c{i};
  else
    a{i} = Xup{i} * a{model.parent(i)} + c{i};
  end
  if fd(i) == 0
    qdd_out(i,1) = qdd(i);
    tau_out(i,1) = S{i}'*(IA{i}*a{i} + pA{i});
  else
    qdd_out(i,1) = (u{i} - U{i}'*a{i})/d{i};
    tau_out(i,1) = tau(i);
    a{i} = a{i} + S{i}*qdd_out(i);
  end
end
