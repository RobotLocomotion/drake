%#codegen
function [q, dq, ddq] = expmap2quatImpl(v) 
  theta = norm(v);
  is_degenerate = (theta <= eps^0.25);
  switch nargout
    case 1
      if is_degenerate
        q = expmap2quatDegenerate(v, theta);
      else
        q = expmap2quatNonDegenerate(v, theta);
      end
    case 2
      if is_degenerate
        [q, dq] = expmap2quatDegenerate(v, theta);
      else
        [q, dq] = expmap2quatNonDegenerate(v, theta);
      end
    case 3      
      if is_degenerate
        [q, dq, ddq] = expmap2quatDegenerate(v, theta);
      else
        [q, dq, ddq] = expmap2quatNonDegenerate(v, theta);
      end
  end
  
end
