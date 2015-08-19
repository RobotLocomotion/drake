function passed = checkForceClosure(contact_pos,contact_normal,mu_face)
% Given a set of contact positions and contact normals, check if we have
% force closure, using the condition described in Boyd's paper 'Fast
% computation of optimal contact forces'
% @param contact_pos   A 3 x num_contacts matrix, contact_pos(:,i) is the
% position of the i'th contact
% @param contact_normal A 3 x num_contacts matrix, contact_normal(:,i) is
% the surface normal vector pointing outward at the i'th contact
% @param mu_face  A scalar. The friction coefficient
num_contacts = size(contact_pos,2);
if(size(contact_pos,1) ~= 3 || any(size(contact_normal) ~= [3,num_contacts]))
  error('contact_pos and contact_normal should both be 3 x %d matrix',num_contacts);
end
contact_normal = contact_normal./bsxfun(@times,ones(3,1),sqrt(sum(contact_normal.^2,1)));
if(numel(mu_face) ~= 1 || mu_face<0)
  error('mu_face should be a non-negative scalar');
end
Z = eye(7)- (1/7)*ones(7);
V = chol(Z);
V = V(1:6,:);
w_ext = sqrt(7/6)*V;

prog = spotsosprog();

% f{i} is a 3 x num_contacts matrix, f{i}(:,j) is the contact force at
% contact_pos{j} to react to wrench disturbance w_ext(:,i);
f = cell(7,1);
for i = 1:7
  [prog,f{i}] = prog.newFree(3,num_contacts);
end

% add the friction cone constraint
for i = 1:7
  for j = 1:num_contacts
    prog = prog.withLor([f{i}(:,j)'*contact_normal(:,j);f{i}(:,j)/sqrt(1+mu_face^2)]);
  end
end

% The contact force can resist the external wrench
G = graspTransferMatrix(contact_pos);
for i = 1:7
  prog = prog.withEqs(w_ext(:,i) - G*reshape(f{i},[],1));
end

options = spot_sdp_default_options();
options.verbose = 0;
solver_sol = prog.minimize(0,@spot_mosek,options);
passed = solver_sol.isPrimalFeasible();
end