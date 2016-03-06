function  fbmodel = floatbase( model )

% floatbase  construct the floating-base equivalent of a fixed-base model
% floatbase(model)  converts a fixed-base kinematic tree to a floating-base
% kinematic tree as follows: old body 1 becomes new body 6, and is regarded
% as the floating base; old joint 1 is discarded; six new joints are added
% (three prismatic and three revolute, in that order, arranged along the x,
% y and z axes, in that order); and five new zero-mass bodies are added
% (numbered 1 to 5), to connect the new joints.  All other bodies and
% joints in the given model are preserved, but their identification numbers
% are incremented by 5.  The end result is that body 6 has a full 6 degrees
% of motion freedom relative to the fixed base, with joint variables 1 to 6
% serving as a set of 3 Cartesian coordinates and 3 rotation angles (about
% x, y and z axes) specifying the position and orientation of body 6's
% coordinate frame relative to the base coordinate frame.  CAUTION: A
% singularity occurs when q(5)=+-pi/2, and these orientations must be
% avoided.  Note: this function requires that old joint 1 is the only joint
% connected to the fixed base, and that Xlink{1} is the identity.

if any( model.parent(2:model.NB) == 0 )
  error('only one connection to a fixed base allowed');
end

if ~isequal( model.Xtree{1}, Xtrans([0,0,0]) )
  warning('Xtree{1} not identity');
end

fbmodel.NB = model.NB + 5;

fbmodel.pitch(1:6) = [inf,inf,inf,0,0,0];
fbmodel.pitch(7:fbmodel.NB) = model.pitch(2:model.NB);

fbmodel.parent(1:6) = [0 1 2 3 4 5];
fbmodel.parent(7:fbmodel.NB) = model.parent(2:model.NB) + 5;

fbmodel.Xtree{1} = Xroty(pi/2);
fbmodel.Xtree{2} = Xrotx(-pi/2) * Xroty(-pi/2);
fbmodel.Xtree{3} = Xrotx(pi/2);
fbmodel.Xtree{4} = Xroty(pi/2);
fbmodel.Xtree{5} = Xrotx(-pi/2) * Xroty(-pi/2);
fbmodel.Xtree{6} = Xrotx(pi/2);

for i = 7:fbmodel.NB
  fbmodel.Xtree{i} = model.Xtree{i-5};
end

for i = 1:fbmodel.NB
  if i < 6
    fbmodel.I{i} = mcI( 0, [0,0,0], zeros(3) );
  else
    fbmodel.I{i} = model.I{i-5};
  end
end

fbmodel.appearance{1} = model.appearance{1};

for i = 6:fbmodel.NB
  fbmodel.appearance{i+1} = model.appearance{i-4};
end
