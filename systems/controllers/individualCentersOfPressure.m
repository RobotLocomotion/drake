function cops = individualCentersOfPressure(r, kinsol, active_supports, normals, B, beta)

n_basis_vectors_per_contact = length(beta) / size(normals, 2);
n = length(active_supports);

normals_start = 0;
indexB = 1;
beta_start = 0;

cops = nan(3, n);

for j = 1:length(active_supports)
  contact_positions = r.getBody(active_supports(j)).getTerrainContactPoints();
  ncj = size(contact_positions, 2);
  normalsj = normals(:, normals_start + (1 : ncj));
  normal = normalsj(:, 1);
  normals_identical = ~any(any(bsxfun(@minus, normalsj, normal)));
  
  if normals_identical % otherwise computing a COP doesn't make sense
    force = zeros(3, 1);
    torque = zeros(3, 1);
    for k = 1 : ncj
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
    cop_body = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
    cops(:, j) = r.forwardKin(kinsol, active_supports(j), cop_body, 0);
  end
  
  normals_start = normals_start + ncj;
end

end