function PP=piecewiseCubicPoly(x,in_y,in_ydot_plus,in_ydot_minus)
% piecewiseCubicPoly(x,y,ydot_plus,ydot_minus) returns a piecewise
% cubic polynomial (matlab pp form) consisting of polynomials that
% match the function values y at break points x, and match the
% derivative values ydot_plus on the left and ydot_minus on the right.
global fullCoeffs strides

N=length(x);
x=reshape(x,1,N);
        
        if nargin==3
            error('do not accept 3 inputs at this time, but will try to add that soon')
        elseif nargin==4
%             ydot_plus=reshape(in_ydot_plus,N-1,1);
%             ydot_minus=reshape(in_ydot_minus,N-1,1);
%             y1dot=ydot_plus;
%             y2dot=ydot_minus;
        else
            error('Incorrect number of inputs!')
        end

y_size=size(in_y);
strides=cumprod([1 y_size]);
y_dim=length(y_size)-1;

if y_size(end)~=N
    error('Need as many y entries as you have break points x!')
end



    curInd=zeros(1,y_dim);
    fullCoeffs=zeros([prod(y_size(1:y_dim)) N-1 4]);
    
    recursiveSplineLooper(1,curInd,y_size,y_dim,x,in_y,in_ydot_plus,in_ydot_minus);

PP=mkpp(x,fullCoeffs,y_size(1:end-1));
end

function recursiveSplineLooper(depth,curInd,y_size,y_dim,x,in_y,in_ydot_plus,in_ydot_minus)
        global fullCoeffs strides
            for i=1:y_size(depth)
                curInd(depth)=i;
                
                if depth==y_dim
                    thisEl=(curInd-1)*strides(1:end-2)'+1;
                    fullCoeffs(thisEl,:,:)=getOneSplineCoeff...
                        (x,in_y(thisEl:strides(end-1):end),...
                        in_ydot_plus(thisEl:strides(end-1):end),...
                        in_ydot_minus(thisEl:strides(end-1):end));
                else
                    recursiveSplineLooper...
                        (depth+1,curInd,y_size,y_dim,x,in_y,in_ydot_plus,in_ydot_minus)
                end
            end
            
                function C=getOneSplineCoeff(x,y,y1dot,y2dot)
        x1=x(1:end-1);
        x2=x(2:end);
        y1=y(1:end-1);
        y2=y(2:end);
        
        coeffs=fitCubePoly(x1,x2,y1,y2,y1dot,y2dot);
        
        c0=coeffs(1:4:end);
        c1=coeffs(2:4:end);
        c2=coeffs(3:4:end);
        c3=coeffs(4:4:end);
        C=full([c3 c2 c1 c0]);
    end
end

function v=fitCubePoly(x1,x2,y1,y2,y1dot,y2dot)
N=length(x1);
A=sparse(4*N,4*N);

b=sparse(1,4*N);
b(1:4:end)=y1;
b(2:4:end)=y2;
b(3:4:end)=y1dot;
b(4:4:end)=y2dot;

tmpInd=sub2ind([4*N 4*N], 1:4:4*N, 1:4:4*N);
A(tmpInd)=1;
tmpInd=sub2ind([4*N 4*N], 2:4:4*N, 1:4:4*N);
A(tmpInd)=1;

%leave 3,1 and 4,1 as zeros

tmpInd=sub2ind([4*N 4*N], 2:4:4*N, 2:4:4*N);
A(tmpInd)=x2-x1;

tmpInd=sub2ind([4*N 4*N], 3:4:4*N, 2:4:4*N);
A(tmpInd)=1;
tmpInd=sub2ind([4*N 4*N], 4:4:4*N, 2:4:4*N);
A(tmpInd)=1;

tmpInd=sub2ind([4*N 4*N], 2:4:4*N, 3:4:4*N);
A(tmpInd)=(x2-x1).^2;

tmpInd=sub2ind([4*N 4*N], 4:4:4*N, 3:4:4*N);
A(tmpInd)=2*(x2-x1);

tmpInd=sub2ind([4*N 4*N], 2:4:4*N, 4:4:4*N);
A(tmpInd)=(x2-x1).^3;

tmpInd=sub2ind([4*N 4*N], 4:4:4*N, 4:4:4*N);
A(tmpInd)=3*(x2-x1).^2;

v=A\(b');
end