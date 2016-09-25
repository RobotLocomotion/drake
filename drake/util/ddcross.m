function ddr1crossr2 = ddcross(r1,r2,dr1,dr2,ddr1,ddr2)
% dr1crossr2 = dcross(r1,r2,dr1,dr2) Compute Jacobian of cross-product
% @param r1 - [3 x 1] array
% @param r2 - [3 x 1] array
% @param dr1 - gradient of r1
% @param dr2 - gradient of r2
% @retval dr1crossr2 - gradient of cross(r1,r2)

% ddr1crossr2 = bsxfun(@cross,r1,dr2) - bsxfun(@cross,r2,dr1);
ddr1crossr2 = bsxfun(@cross,ddr1,r2) + 2*reshape([bsxfun(@cross,dr1,dr2(:,1)); bsxfun(@cross,dr1,dr2(:,2)); bsxfun(@cross,dr1,dr2(:,3))],3,[]) + bsxfun(@cross,r1,ddr2);
end