function ret = vectorToSkewSymmetric(p)
%VECTORTOSKEWSYMMETRIC Computes the skew symmetric matrix
% [0, -pz, py;
%  pz, 0, -px;
% -py, px, 0];
% given vector [px; py; pz]

px = p(1);
py = p(2);
pz = p(3);

ret = [0, -pz, py;
       pz, 0, -px;
       -py, px, 0];
end