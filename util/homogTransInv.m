function TInv = homogTransInv(T)
TInv = 0*T; % for symbolic
R = T(1:3, 1:3);
p = T(1:3, 4);
TInv(1:3, 1:3) = R';
TInv(1:3, 4) = -R' * p;
TInv(4, 4) = 1;
end