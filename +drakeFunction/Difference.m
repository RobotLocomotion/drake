function fcn = Difference(frame)
  typecheck(frame,'CoordinateFrame');
  input_frame = MultiCoordinateFrame({frame,frame});
  output_frame = frame;
  A = [eye(frame.dim),-eye(frame.dim)];
  fcn = drakeFunction.Linear(input_frame,output_frame,A);
end
