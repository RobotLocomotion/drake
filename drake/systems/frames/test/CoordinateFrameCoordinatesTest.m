classdef CoordinateFrameCoordinatesTest < matlab.unittest.TestCase
  %NOTEST
  methods (Test)
    
    
    function testSpecifiedCoordinateNames (testCase)
      
      % tests that we can work with the coordinate names of a CoordinateFrame
      
      f_set = CoordinateFrame('test1', 4, 'y', {'a', 'b', 'c', 'd'}); % coordinate names are set
      testCase.assertEqual(f_set.getCoordinateName(2), 'b');
      testCase.assertEqual(f_set.findCoordinateIndex('b'), 2);
      testCase.assertEqual(f_set.getCoordinateNames, {'a', 'b', 'c', 'd'}');
      
      expectedSubFrame = f_set.subFrame([1 3]);
      actualSubFrame = CoordinateFrame('test1[1 3]', 2, ['y', 'y'], {'a', 'c'});
      testCase.assertEqual(expectedSubFrame, actualSubFrame);
      
    end
    
    function testGeneratedCoordinateNames (testCase)
      f_generated = CoordinateFrame('test2', 4, 'z'); % coordinate names generated
      testCase.assertEqual(f_generated.getCoordinateName(2), 'z2');
    end
    
  end
end




