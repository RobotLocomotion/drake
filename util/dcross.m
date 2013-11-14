function dr1crossr2 = dcross(r1,r2)
% dr1crossr2 = dcross(r1,r2,dr1,dr2) Compute Jacobian of cross-product
% @param r1 - [3 x 1] array
% @param r2 - [3 x 1] array
% @retval dr1crossr2 - Jacobian of cross(r1,r2) wrt [r1;r2] 

dr1 = [eye(3),zeros(3)];
dr2 = [zeros(3),eye(3)];
dr1crossr2 = bsxfun(@cross,r1,dr2) - bsxfun(@cross,r2,dr1); 
end
