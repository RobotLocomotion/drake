function w = flipExpmap(w)
% flip the exponential map 
w_norm = norm(w);
if(w_norm>eps)
  w_dir = w/w_norm;
  w = w_dir*-2*pi-w;
end
end