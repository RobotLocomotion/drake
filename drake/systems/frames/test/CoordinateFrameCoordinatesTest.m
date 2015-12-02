classdef CoordinateFrameCoordinatesTest < matlab.unittest.TestCase
  %NOTEST
  
  % Tests that functions that touch the Coordinates property of
  % CoordinateFrame work.
  
  properties
    SpecifiedFrame
    GeneratedFrame
  end
  
  methods(TestMethodSetup)
    function createFrames (tc)
      % SpecifiedFrame has coordinate names set by the user directly
      tc.SpecifiedFrame = CoordinateFrame('test1', 4, 'y', {'a', 'b', 'c', 'd'});
      
      % GeneratedFrame has coordinate names that are
      % automatically generated from the prefix provided.
      tc.GeneratedFrame = CoordinateFrame('test2', 4, 'z');
    end
  end
  
  methods(Test)
    
    function testGetCoordinateName (tc)
      tc.assertEqual(tc.SpecifiedFrame.getCoordinateName(2), 'b');
      tc.assertEqual(tc.GeneratedFrame.getCoordinateName(2), 'z2');
    end
    
    function testFindCoordinateIndex (tc)
      tc.assertEqual(tc.SpecifiedFrame.findCoordinateIndex('b'), 2);
      tc.assertEqual(tc.GeneratedFrame.findCoordinateIndex('z2'), 2);
    end
    
    function testGetCoordinateNames (tc)
      tc.assertEqual(tc.SpecifiedFrame.getCoordinateNames, {'a', 'b', 'c', 'd'}');
      tc.assertEqual(tc.GeneratedFrame.getCoordinateNames, {'z1', 'z2', 'z3', 'z4'}');
    end
    
    function testSubFrameSpecifiedFrame (tc)
      % ensures that we extract the appropriate subFrame
      expectedSubFrame = tc.SpecifiedFrame.subFrame([1 3]);
      actualSubFrame = CoordinateFrame('test1[1 3]', 2, ['y', 'y'], {'a', 'c'});
      tc.assertEqual(expectedSubFrame, actualSubFrame);
    end
    
    function testSubFrameGeneratedFrame (tc)
      expectedSubFrame = tc.GeneratedFrame.subFrame([1 3]);
      actualSubFrame = CoordinateFrame('test2[1 3]', 2, ['z', 'z'], {'z1', 'z3'});
      tc.assertEqual(expectedSubFrame, actualSubFrame);
    end
    
    % TODO
    % Functions to write tests for: disp, isequal_modulo_transforms,
    % getSym, addProjectionTransformByCoordinateNames,
    % constructFrameWithAnglesWrapped, scope, generateLCMType,
    % setCoordinates
    
  end
end




