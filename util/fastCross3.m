function X = fastCross3(U, V)
% cross() is ludicrously slow. This is about 10x faster for 3-vectors, but does no
% input checking. Test case in test/testFastCross3.m

X = [U(2) * V(3) - U(3) * V(2);
     U(3) * V(1) - U(1) * V(3);
     U(1) * V(2) - U(2) * V(1)];