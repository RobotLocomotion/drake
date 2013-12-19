function ret = twistToTildeForm(twist)
omega = twist(1 : 3);
v = twist(4 : 6);
ret = [vectorToSkewSymmetric(omega), v;
       zeros(1, 4)];
end