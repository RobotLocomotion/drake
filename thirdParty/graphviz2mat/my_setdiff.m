function C = my_setdiff(A,B)
% MYSETDIFF Set difference of two sets of positive integers (much faster than built-in setdiff)
% C = my_setdiff(A,B)
% C = A \ B = { things in A that are not in B }

%  by Leon Peshkin pesha at ai.mit.edu 2004, inspired by BNT of Kevin Murphy
if isempty(A)
    C = [];
    return;
elseif isempty(B)
    C = A;
    return; 
else % both non-empty
    bits = zeros(1, max(max(A), max(B)));
    bits(A) = 1;
    bits(B) = 0;
    C = A(logical(bits(A)));
end