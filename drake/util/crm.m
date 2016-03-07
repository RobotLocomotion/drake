function vcross = crm(v)

% spatial cross-product operator (motion)

vcross = [ vectorToSkewSymmetric(v(1:3)), zeros(3); ...
           vectorToSkewSymmetric(v(4:6)), vectorToSkewSymmetric(v(1:3)) ];
         