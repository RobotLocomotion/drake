function p = legendrePoly(n)
% Return the a vector representing the legendre polynomial, use polyval for
% polynomial evaluation.
if(any(size(n)~=[1,1]))
    error('The input order must be a scalar');
end
if(n<0||n~=int32(n))
    error('The input order must be a nonnegative integer')
end
if(n==0)
    p = [1];
elseif(n==1)
    p = [1 0];
else
    p_i_1 = [1 0];
    p_i_2 = [1];
    for i = 2:n
        p_i = ((2*i-1)*[p_i_1 0]-(i-1)*[0 0 p_i_2])/i;
        p_i_2 = p_i_1;
        p_i_1 = p_i;
    end
    p = p_i;
end
