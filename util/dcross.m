function dr1crossr2 = dcross(r1, r2, dr1, dr2)
% dr1crossr2 = dcross(r1,r2,dr1,dr2) Compute Jacobian of cross-product
% @param r1 - [3 x 1] array
% @param r2 - [3 x 1] array
% @param dr1 - gradient of r1
% @param dr2 - gradient of r2
% @retval dr1crossr2 - gradient of cross(r1,r2)

dr1crossr2 = bsxfun(@cross,r1,dr2) - bsxfun(@cross,r2,dr1);
end