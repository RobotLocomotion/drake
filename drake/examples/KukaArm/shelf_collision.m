function phi = shelf_collision(p,x)
    nq = 7;
    q = x(1:nq);
    kinsol = doKinematics(p, q);
    phi = p.contactConstraints(kinsol);
end

