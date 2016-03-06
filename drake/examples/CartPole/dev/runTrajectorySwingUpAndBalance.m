function runTrajectorySwingUpAndBalance

p = CartPolePlant();
p = setInputLimits(p,-inf,inf);

c = trajectorySwingUpAndBalance(p);

