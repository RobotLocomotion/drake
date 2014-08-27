classdef ControlSurface
  
  properties
    name;
    chord;
    span;
    left_edge_position_along_wing;
    min_deflection; % in radians
    max_deflection; % in radians
    
  end
  
  
  methods
    
    function obj = ControlSurface(name, chord, span, left_edge_position_along_wing, min_deflection, max_deflection)
      obj.name = name;
      obj.chord = chord;
      obj.span = span;
      obj.left_edge_position_along_wing = left_edge_position_along_wing;
      obj.min_deflection = min_deflection;
      obj.max_deflection = max_deflection;
    end
    
  end
  
  
end
    