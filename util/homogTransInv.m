function HInv = homogTransInv(H)
HInv = zeros(4, 4);
R = H(1:3, 1:3);
p = H(1:3, 4);
HInv(1:3, 1:3) = R';
HInv(1:3, 4) = -R' * p;
HInv(4, 4) = 1;
end