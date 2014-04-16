function HInv = homogTransInv(H)
HInv = 0*H; % for symbolic
R = H(1:3, 1:3);
p = H(1:3, 4);
HInv(1:3, 1:3) = R';
HInv(1:3, 4) = -R' * p;
HInv(4, 4) = 1;
end