function cops = individualCentersOfPressure(r, kinsol, active_supports, normals, B, beta)

n_basis_vectors_per_contact = length(beta) / size(normals, 2);

normals_start = 0;
indexB = 1;
beta_start = 0;

cops = nan(3, length(active_supports));

% everything in world frame.
for j = 1:length(active_supports)
  contact_positions = r.getBody(active_supports(j)).getTerrainContactPoints();
  contact_positions = r.forwardKin(kinsol, active_supports(j), contact_positions);
  num_contacts_j = size(contact_positions, 2);
  normals_j = normals(:, normals_start + (1 : num_contacts_j));
  normal = normals_j(:, 1);
  normals_identical = ~any(any(bsxfun(@minus, normals_j, normal) > 1e-8));
  
  if normals_identical
    force = zeros(3, 1);
    torque = zeros(3, 1);
    for k = 1 : num_contacts_j
      Bblock = B{indexB};
      betablock = beta(beta_start + (1 : n_basis_vectors_per_contact));
      contact_position = contact_positions(:, k);
      point_force = Bblock * betablock;
      force = force + point_force;
      point_torque = cross(contact_position, point_force);
      torque = torque + point_torque;
      indexB = indexB + 1;
      beta_start = beta_start + n_basis_vectors_per_contact;
    end
    
    point_on_contact_plane = contact_positions(:, 1);
    cops(:, j) = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
  else
    error('normals must be identical to compute CoP');  % otherwise computing a COP doesn't make sense
  end
  
  normals_start = normals_start + num_contacts_j;
end

end