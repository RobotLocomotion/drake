function quat_norm = quaternionDistance(quat1,quat2)
lambda = sum(bsxfun(@times,quat1,quat2),1);
quat_norm = 1-abs(lambda);
end
