function I_new = moveSpatialMassMatrix(c,I_old)
% Moves an added mass matrix (or spatial mass matrix) by c.
% Equivalent to mcI(m,c,Icm) but works for non-scalar m and
% allows for inertias to not be about center of mass
% @param I_old is spatial mass matrix in [angular;velo] form as
% consistent with Featherstone's notation
% @param c [3x1] is the vector from the NEW origin to the OLD origin

M11 = I_old(1:3,1:3); M12 = I_old(1:3,4:6);
M21 = I_old(4:6,1:3); M22 = I_old(4:6,4:6);
C = [  0,    -c(3),  c(2);
    c(3),  0,    -c(1);
    -c(2),  c(1),  0 ];

% Version in usual [velo,angular] coordinates
%I_new = [M11 -M11*C+M12; C*M11+M21 M22-C*M11*C-M21*C+C*M12];

% Version in Featherstone's notation: swaps M11 for M22 and M12 for M21,
% and order of elements
I_new = [M11-C*M22*C-M12*C+C*M21 C*M22+M12; -M22*C+M21 M22];
 
end