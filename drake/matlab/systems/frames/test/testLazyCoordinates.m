function testLazyCoordinates

  % Tests that functions that touch the coordinates property of
  % CoordinateFrame work.
  
  % specified_frame has coordinate names set by the user directly
  specified_frame = CoordinateFrame('test1', 4, 'y', {'a', 'b', 'c', 'd'});
      
  % generated_frame has coordinate names that are
  % automatically generated from the prefix provided.
  generated_frame = CoordinateFrame('test2', 4, 'z');

  %%
  assert(isequal(specified_frame.getCoordinateName(2), 'b'));
  assert(isequal(generated_frame.getCoordinateName(2), 'z2'));

  assert(isequal(specified_frame.findCoordinateIndex('b'), 2));
  assert(isequal(generated_frame.findCoordinateIndex('z2'), 2));

  assert(isequal(specified_frame.getCoordinateNames, {'a', 'b', 'c', 'd'}'));
  assert(isequal(generated_frame.getCoordinateNames, {'z1', 'z2', 'z3', 'z4'}'));

  % ensures that we extract the appropriate subFrame
  expectedSubFrame = specified_frame.subFrame([1 3]);
  actualSubFrame = CoordinateFrame('test1[1 3]', 2, ['y', 'y'], {'a', 'c'});
  assert(isequal(expectedSubFrame, actualSubFrame));

  expectedSubFrame = generated_frame.subFrame([1 3]);
  actualSubFrame = CoordinateFrame('test2[1 3]', 2, ['z', 'z'], {'z1', 'z3'});
  assert(isequal(expectedSubFrame, actualSubFrame));

  % TODO
  % Functions to write tests for: disp, isequal_modulo_transforms,
  % getSym, addProjectionTransformByCoordinateNames,
  % constructFrameWithAnglesWrapped, scope, generateLCMType,
  % setCoordinates
  
end




