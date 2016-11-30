function dAB=matGradMultMat(A,B,dA,dB)

% takes the gradient of the A(q)*B(q), by doing dA(q)*B(q) + A(q)*dB(q),
% where dA and dB are matrices specified in the geval grad format

[m,n] = size(A);
[n2,p] = size(B); 
if (n~=n2) error('dimension mismatch'); end
[mn,q] = size(dA);
if (mn ~= m*n) error('dimension mismatch'); end
[np,q2] = size(dB);
if (np ~= n*p) error('dimension mismatch'); end
if (q2 ~= q) error('dA and dB must have the same number of gradient terms'); end

% speye is faster, so if we have numerical inputs we should use that. We
% can't use speye with TaylorVars, however, because they don't implement
% "find". We're switching based on `isnumeric` rather than
% `isa(...,'TaylorVar') to avoid doing a string comparision.
if isnumeric(B)
  B_diag = kron(speye(q),B);
else
  B_diag = kron(eye(q),B);
end
dAB = reshape(reshape(dA,m,n*q)*B_diag + A*reshape(dB,n,p*q),m*p,q);


% an alternative for the second half is what i did in the contact code:
        % for a single contact, we'd have
        % n = normal'*J;
        % For vectorization, I just construct
        %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0], 
        % etc, where each 0 is a 1x3 block zero, then multiply by J

%        n = sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,normal(:))*J;
