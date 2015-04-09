%#codegen
function [q, dq, ddq] = exp2quatImpl(v) 
  theta = norm(v);
  is_degenerate = (theta <= eps^0.25);
  switch nargout
    case 1
      if is_degenerate
        q = exp2quatDegenerate(v, theta);
      else
        q = exp2quatNonDegenerate(v, theta);
      end
    case 2
      if is_degenerate
        [q, dq] = exp2quatDegenerate(v, theta);
      else
        [q, dq] = exp2quatNonDegenerate(v, theta);
      end
    case 3      
      if is_degenerate
        [q, dq, ddq] = exp2quatDegenerate(v, theta);
      else
        [q, dq, ddq] = exp2quatNonDegenerate(v, theta);
      end
  end
  
end
