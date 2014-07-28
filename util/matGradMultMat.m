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

B_diag_row = reshape(bsxfun(@times,reshape(1:n2*q,n2,1,q),ones(1,p,1)),[],1);
B_diag_col = reshape(bsxfun(@times,ones(n2,1),(1:p*q)),[],1);
B_diag_val = reshape(bsxfun(@times,B(:),ones(1,q)),[],1);
B_diag = sparse(B_diag_row,B_diag_col,B_diag_val,n2*q,p*q);
dAB = reshape(reshape(dA,m,n*q)*B_diag + A*reshape(dB,n,p*q),m*p,q);


% an alternative for the second half is what i did in the contact code:
        % for a single contact, we'd have
        % n = normal'*J;
        % For vectorization, I just construct
        %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0], 
        % etc, where each 0 is a 1x3 block zero, then multiply by J

%        n = sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,normal(:))*J;
