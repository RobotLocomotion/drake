function tf = polycone(v, r, N)
% Polygonal approximation of the conic constraint norm(v) <= r in two dimensions. Uses an N-sided regular polygon circumscribed about the circle of radius r. 
% @retval tf a logical vector of length N. The constraint is satisfied if
%            all(tf) is true. This slightly unusual form means that polycone
%            can be used as a drop-in replacement for the Yalmip cone()
%            function. 

sizecheck(v, [2, nan]);
sizecheck(r, 1);
sizecheck(N, 1);
A = zeros(N, 2);
b = repmat(r, N, 1);
for j = 1:N
  th = (2*pi) / N * (j-1);
  R = rotmat(th);
  A(j,:) = (R * [0;1])';
end
tf = A * v <= repmat(b, 1, size(v, 2));

