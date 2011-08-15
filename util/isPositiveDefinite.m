function isposdef = isPositiveDefinite(M)

isposdef = true;
for i=1:length(M)
  if ( det( M(1:i, 1:i) ) <= 0 )
    isposdef = false;
    break;
  end
end
