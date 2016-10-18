function vcross = crf(v)

% spatial cross-product operator (force)

vcross = [ vectorToSkewSymmetric(v(1:3)), vectorToSkewSymmetric(v(4:6)); ...
           zeros(3), vectorToSkewSymmetric(v(1:3)) ];
         