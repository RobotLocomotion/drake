function dr1crossr2 = dcross(r1,r2)
% dr1crossr2 = dcross(r1,r2,dr1,dr2) Compute Jacobian of cross-product
% @param r1 - [3 x 1] array
% @param r2 - [3 x 1] array
% @retval dr1crossr2 - Jacobian of cross(r1,r2) wrt [r1;r2]

% dr1 = [eye(3),zeros(3)];
% dr2 = [zeros(3),eye(3)];
% dr1crossr2 = bsxfun(@cross,r1,dr2) - bsxfun(@cross,r2,dr1);
dr1crossr2 = [0 r2(3) -r2(2) 0 -r1(3) r1(2);...
              -r2(3) 0 r2(1) r1(3) 0 -r1(1);...
              r2(2) -r2(1) 0 -r1(2) r1(1) 0];
end