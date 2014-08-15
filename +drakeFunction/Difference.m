function fcn = Difference(frame,n)
  typecheck(frame,'CoordinateFrame');
  integervaluedcheck(n);
  assert(n>=2,'Drake:DrakeFunction:Difference:BadInput',...
    'The number of points to difference, n, must be at least 2');
  input_frame = MultiCoordinateFrame.constructFrame(repmat({frame},1,n));
  output_frame = frame;
  nx = frame.dim;
  A = kron(spdiags(ones(nx,1),0,n-1,n),eye(nx)) + kron(spdiags(-ones(nx,1),1,n-1,n),eye(nx));
  fcn = drakeFunction.Linear(input_frame,output_frame,A);
end
