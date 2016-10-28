function C = comm(n,m)
%COMM Returns the commutator matrix C_{n,m} that turns vec(A) into vec(A')
%where A is an n x m matrix.

if (n*m)*(n*m) > 100 %decide if C should be sparse or not
    C = reshape(kron(vec(speye(n)), speye(m)), n*m, n*m);
else
    C = reshape(kron(vec(eye(n)), eye(m)), n*m, n*m);
end

end

