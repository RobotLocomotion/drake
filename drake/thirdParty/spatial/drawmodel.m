function  drawmodel( model, delta_t, jnt_vals )
  
% drawmodel  display system model or animation
% drawmodel(model)  will display a system model, and
% drawmodel(model,delta_t,jnt_vals)  will show an animation.
% Both calls create a text file in the current directory, and ty to display
% it by calling the program anim8 (1st choice) or SceneViewer (2nd choice).
% The first call creates a file called data_km.iv that depicts the model's
% appearance and kinematics.  The second call creates a file called
% data_mv.iv that depicts the motion described by jnt_vals.  Arguments:
% model is a system-model data structure; jnt_vals is a 2-dimensional array
% with as many rows as data points, and as many columns as joints in the
% system model; and delta_t is the time step between consecutive data
% points.

if nargin ~= 1 && nargin ~= 3
  error('wrong number of arguments');
end

if ~isfield(model,'appearance')
  error('no appearance data in model');
end

if nargin == 3
  if length(delta_t) > 1 || ~isnumeric(delta_t) || delta_t <= 0
    error('delta_t must be a number greater than zero');
  end
  if size(jnt_vals,2) ~= model.NB
    error('# columns in jnt_vals must equal # joints in model');
  end
end

if nargin == 1
  file = 'data_km.iv';
else
  file = 'data_mv.iv';
end

fid = fopen(file,'w');

if fid == -1
  error('could not open file %s\n', file);
end

fprintf( fid, '#Inventor V2.1 ascii\n');
fprintf( fid, '# drawmodel output for anim8\n\n');

if nargin == 1
  ok = treedraw( model, fid, 0 );
else
  ok = motiondraw( model, delta_t, jnt_vals, fid, 0 );
end

if fclose(fid) ~= 0
  error('could not close file %s\n', file);
end

if ~isunix
  warning('drawing file %s can be displayed only under unix', file);
  return
end

[status,result] = system('which anim8');
if status == 0
  program = 'anim8';
else
  program = 'SceneViewer';
end

if ok
  command = sprintf('%s %s', program, file);
  [status,result] = system(command);
  if status ~= 0
    error('error running %s: %s\n', program, result);
  end
end




% Recursively draw a kinematic tree

function ok = treedraw( model, fid, r )

fprintf( fid, 'Separator {\n');

if r == 0				% output preamble

  fprintf( fid, '  ShapeHints {\n');
  fprintf( fid, '    vertexOrdering COUNTERCLOCKWISE\n');
  fprintf( fid, '    shapeType SOLID\n');
  fprintf( fid, '    faceType UNKNOWN_FACE_TYPE\n');
  fprintf( fid, '  }\n' );

  % output joint variable controls in numerical order

  for i = 1:model.NB
    fprintf( fid, '  Info { string "anim8: Control j%d', i );
    if model.pitch(i) == 0
      fprintf( fid, '.angle Range -3.14159 3.14159" }\n' );
    elseif model.pitch(i) == inf
      fprintf( fid, '.z[0] Range -1 1" }\n' );
    else
      fprintf( fid, '_angle Range -6.28319 6.28319" }\n' );
    end
  end

else					% output joint transform

  xform( model.Xtree{r}, fid );

  if model.pitch(r) == 0		% revolute joint

    text = '  DEF j%d RotationXYZ { axis Z angle 0 }\n';

    fprintf( fid, text, r );

  elseif model.pitch(r) == inf		% prismatic joint

    text = [...
    '  Translation { translation = DEF j%d ComposeVec3f { z 0 }.vector }\n'...
    ];

    fprintf( fid, text, r );

  else					% helical joint

    text = [...
    '  Translation {\n' ...
    '    translation =\n' ...
    '      Calculator {\n' ...
    '        a = DEF j%d GlobalField { type "SFFloat"'...
				     ' j%d_angle 0 } .j%d_angle\n'...
    '        expression "oA = vec3f(0,0,%.7g*a)"\n'...
    '      } .oA\n'...
    '  }\n'...
    '  RotationXYZ { axis Z angle = USE j%d.j%d_angle }\n'...
    ];

    fprintf( fid, text, r, r, r, model.pitch(r), r, r );

  end

end

fprintf( fid, 'Separator {\n' );	% draw the link
fprintf( fid, '  %s\n', colour_text(r+1) );
ok = linkgeo( model.appearance{r+1}, fid );
fprintf( fid, '}\n' );

for i = (r+1):model.NB			% recursively draw the children
  if ~ok
    return
  end
  if model.parent(i) == r
    ok = treedraw(model, fid, i);
  end
end

fprintf( fid, '}\n' );




% Recursively draw an animation

function ok = motiondraw( model, delta_t, jnt_vals, fid, r )

fprintf( fid, 'Separator {\n' );

if r == 0				% output preamble

  fprintf( fid, '  ShapeHints {\n');
  fprintf( fid, '    vertexOrdering COUNTERCLOCKWISE\n');
  fprintf( fid, '    shapeType SOLID\n');
  fprintf( fid, '    faceType UNKNOWN_FACE_TYPE\n');
  fprintf( fid, '  }\n' );
  fprintf( fid, '  Info { string "anim8: Tmax %g" }\n',...
			  (size(jnt_vals,1) - 1) * delta_t);

else					% output joint transform

  xform(model.Xtree{r}, fid);

  if model.pitch(r) == 0		% revolute joint

    fprintf( fid, '  RotationXYZ { axis Z angle =\n' );
    engine( jnt_vals(:,r), delta_t, r, (r==1), fid );
    fprintf( fid, '  }\n' );

  elseif model.pitch(r) == inf		% prismatic joint

    fprintf( fid, '  Translation { translation = ComposeVec3f { z =\n' );
    engine( jnt_vals(:,r), delta_t, r, (r==1), fid );
    fprintf( fid, '  } .vector }\n' );

  else					% helical joint

    text = [...
    '  Translation { translation =\n' ...
    '    Calculator {\n' ...
    '      expression "oA = vec3f(0,0,%.7g*a)"\n'...
    '      a = DEF j%d GlobalField { type "SFFloat" j%d_angle =\n'
    ];
    fprintf( fid, text, model.pitch(r), r, r );
    engine( jnt_vals(:,r), delta_t, r, (r==1), fid );
    text = [...
    '      } .j%d_angle\n'...
    '    } .oA\n'...
    '  }\n'...
    '  RotationXYZ { axis Z angle = USE j%d.j%d_angle }\n'...
    ];
    fprintf( fid, text, r, r, r );

  end

end

fprintf( fid, 'Separator {\n' );	% draw the link
fprintf( fid, '  %s\n', colour_text(r+1) );
ok = linkgeo( model.appearance{r+1}, fid );
fprintf( fid, '}\n' );

for i = (r+1):model.NB			% recursively draw the children
  if ~ok
    return
  end
  if model.parent(i) == r
    ok = motiondraw(model, delta_t, jnt_vals, fid, i);
  end
end

fprintf( fid, '}\n' );




function engine( vals, delta_t, r, full, fid )

% This function generates a scalar value (the output of an InterpolateFloat
% engine) that is the value of a single joint variable as a function of time.

fprintf( fid, '    InterpolateFloat {\n' );

if full					% output code for time source

  fprintf( fid, '      alpha = DEF interpDriver\n' );
  fprintf( fid, '        Calculator {\n' );
  fprintf( fid, '          a = ElapsedTime { } .timeOut\n' );
  fprintf( fid, '          b %g\n', (length(vals)-1) * delta_t );
  fprintf( fid, '          c %d\n', length(vals) );
  fprintf( fid, '          expression [\n' );
  fprintf( fid, '            "ta = a - (b+4)*floor((a+2)/(b+4))",\n' );
  fprintf( fid, '            "td = ta * (c-1)/b",\n' );
  fprintf( fid, '            "ta = floor(td)",\n' );
  fprintf( fid, '            "tb = ceil(td)",\n' );
  fprintf( fid, '            "oa = ta<0 ? 0 : ta>c-1 ? c-1 : ta",\n' );
  fprintf( fid, '            "ob = tb<0 ? 0 : tb>c-1 ? c-1 : tb",\n');
  fprintf( fid, '            "tc = td - oa",\n' );
  fprintf( fid, '            "oc = tc<0 ? 0 : tc>1 ? 1 : tc" ]\n');
  fprintf( fid, '        } . oc\n' );

else					% re-use existing time source

  fprintf( fid, '      alpha = USE interpDriver . oc\n' );

end

fprintf( fid, '      input0 =\n' );
fprintf( fid, '        SelectOne {\n' );
fprintf( fid, '          type "MFFloat"\n' );
fprintf( fid, '          index = USE interpDriver .oa\n' );
fprintf( fid, '          input = DEF q%d\n', r );
fprintf( fid, '            GlobalField {\n' );
fprintf( fid, '              type "MFFloat" q%d [\n', r);

for i = 1:length(vals)
  fprintf( fid, '%.7g', vals(i) );
  if i == length(vals)
    fprintf( fid, ' ]\n' );
  else
    if mod(i,5) == 0
      fprintf( fid, ',\n' );
    else
      fprintf( fid, ', ' );
    end
  end
end

fprintf( fid, '            } .q%d\n' , r);
fprintf( fid, '        } .output\n' );
fprintf( fid, '      input1 =\n' );
fprintf( fid, '        SelectOne {\n' );
fprintf( fid, '          type  "MFFloat"\n' );
fprintf( fid, '          index = USE interpDriver .ob\n' );
fprintf( fid, '          input = USE q%d .q%d\n' , r, r);
fprintf( fid, '        } .output\n' );
fprintf( fid, '    } .output\n' );  




% Output drawing instructions for one link

function ok = linkgeo( app, fid )

  ok = 1;				% no errors yet

  pos = [0;0;0];			% track current position

  for i = 1:length(app)

    inst = app{i};			% current instruction

    if strcmp( inst{1}, 'vertex' )

      fprintf( fid, '  Coordinate3 { point [\n' );
      v = inst{2};
      for i = 1:size(v,1)-1
	fprintf( fid, '\t%g %g %g,\n', v(i,:) );
      end
      fprintf( fid, '\t%g %g %g ] }\n', v(size(v,1),:) );

    elseif strcmp( inst{1}, 'line' )

      if max( pos ~= [0;0;0] )
        fprintf( fid, '  Translation { translation %g %g %g }\n', -pos );
        pos = [0;0;0];
      end
      fprintf( fid, '  IndexedLineSet { coordIndex [\n\t' );
      for i = 2:length(inst)
	lin = inst{i};
	for j = 1:length(lin)
	  fprintf( fid, ' %d,', lin(j)-1 );
	  if mod(j,12)==0
	    fprintf( fid, '\n\t' );
	  end
	end
	if i == length(inst)
	  fprintf( fid, ' -1 ] }\n' );
	else
	  fprintf( fid, ' -1,\n\t' );
	end
      end

    elseif strcmp( inst{1}, 'face' )

      if max( pos ~= [0;0;0] )
        fprintf( fid, '  Translation { translation %g %g %g }\n', -pos );
        pos = [0;0;0];
      end
      fprintf( fid, '  IndexedFaceSet { coordIndex [\n\t' );
      for i = 2:length(inst)
	face = inst{i};
	for j = 1:length(face)
	  fprintf( fid, ' %d,', face(j)-1 );
	  if mod(j,12)==0
	    fprintf( fid, '\n\t' );
	  end
	end
	if i == length(inst)
	  fprintf( fid, ' -1 ] }\n' );
	else
	  fprintf( fid, ' -1,\n\t' );
	end
      end

    elseif strcmp( inst{1}, 'box' )

      a = inst{2};
      c = (a(:,1)+a(:,2))/2;		% box centre
      d = a(:,2) - a(:,1);		% box dimensions
      if max( c ~= pos )
        fprintf( fid, '  Translation { translation %g %g %g }\n', c-pos );
        pos = c;
      end
      fprintf( fid, '  Cube { width %g height %g depth %g }\n', d );

    elseif strcmp( inst{1}, 'cyl' )

      c = inst{2};			% centre
      r = inst{3};			% radius
      h = inst{4};			% height
      dir = inst{5};			% direction: X, Y or Z
      if size(c,2) == 3
	c = c';				% make sure it is a column vector
      end
      if max( c ~= pos )
        fprintf( fid, '  Translation { translation %g %g %g }\n', c-pos );
        pos = c;
      end
      if dir == 'X'
        fprintf( fid, '  RotationXYZ { axis Z angle 1.5708 }\n' );
        fprintf( fid, '  Cylinder { radius %g height %g }\n', r, h );
        fprintf( fid, '  RotationXYZ { axis Z angle -1.5708 }\n' );
      elseif dir == 'Z'
        fprintf( fid, '  RotationXYZ { axis X angle 1.5708 }\n' );
        fprintf( fid, '  Cylinder { radius %g height %g }\n', r, h );
        fprintf( fid, '  RotationXYZ { axis X angle -1.5708 }\n' );
      else
        fprintf( fid, '  Cylinder { radius %g height %g }\n', r, h );
      end

    else

      warning( 'ignoring unrecognised drawing instruction: %s\n', inst{1} );

    end
  end




% Colours for automatic colouring scheme.  Colours are numbered from 1
% (the colour for the base), and repeat cyclically when n > # colours

function colour = colour_text( n )

colours = { ...
  'Material { diffuseColor 0.7 0.1 0.1 ambientColor 0.35 0.05 0.05 }',...
  'Material { diffuseColor 0.8 0.45 0.1 ambientColor 0.4 0.22 0.05 }',...
  'Material { diffuseColor 0.8 0.7 0.1 ambientColor 0.4 0.35 0.05 }',...
  'Material { diffuseColor 0.5 0.7 0.2 ambientColor 0.25 0.35 0.1 }',...
  'Material { diffuseColor 0.1 0.6 0.1 ambientColor 0.05 0.3 0.05 }',...
  'Material { diffuseColor 0.1 0.75 0.75 ambientColor 0.05 0.37 0.37 }',...
  'Material { diffuseColor 0.1 0.45 0.8 ambientColor 0.05 0.22 0.4 }',...
  'Material { diffuseColor 0.6 0.25 0.8 ambientColor 0.3 0.12 0.4 }',...
  'Material { diffuseColor 0.8 0.5 0.8 ambientColor 0.4 0.25 0.4 }',...
  'Material { diffuseColor 0.8 0.1 0.6 ambientColor 0.4 0.05 0.3 }',...
  'Material { diffuseColor 0.6 0.6 0.55 ambientColor 0.3 0.3 0.27 }',...
  'Material { diffuseColor 0.55 0.4 0.25 ambientColor 0.27 0.2 0.12 }' };

colour = colours{ mod( n-1, length(colours)) + 1 };




% Output the instructions to perform the given transform

function xform(X, fid)

  rcrosst = X(1:3,1:3)' * X(4:6,1:3);

  rx = rcrosst(2,3);
  ry = rcrosst(3,1);
  rz = rcrosst(1,2);

  if rx ~= 0 || ry ~= 0 || rz ~= 0
    fprintf( fid, '  Translation { translation %.7g %.7g %.7g }\n',...
					       rx, ry, rz);
  end

  axis = m3unrotv(X(1:3,1:3));

  angle = sqrt(dot(axis,axis));

  if angle > 1e-12
    fprintf( fid, '  Rotation { rotation %g %g %g %.7g }\n', axis, angle);
  end




% This function, transcribed from Roy Featherstone's mat3 library, calculates
% a vector whose magnitude and direction are the angle and axis of the
% rotation represented by the orthonormal matrix E

function v = m3unrotv(E)

  RT_RESOL = 4.5e-9;
  SMALL = 1e-4;

  v(1) = E(2,3) - E(3,2);
  v(2) = E(3,1) - E(1,3);
  v(3) = E(1,2) - E(2,1);

  S = dot(v,v);

  tr1 = trace(E) - 1;

  if S < RT_RESOL && tr1 > 0

    S = (S + 24) / 48;
    v = v * S;

  elseif S < SMALL && tr1 < 0

    S = atan2(sqrt(S), tr1);

    if E(1,1) > E(2,2) && E(1,1) > E(3,3)

      d = 2 * E(1,1) - tr1;
      v(1) = S * sqrt( d / (2 - tr1) );

      if E(3,2) > E(2,3)
        v(1) = -v(1);
      end

      v(2) = (E(1,2) + E(2,1)) * v(1) / d;
      v(3) = (E(1,3) + E(3,1)) * v(1) / d;

    elseif E(2,2) > E(3,3)

      d = 2 * E(2,2) - tr1;
      v(2) = S * sqrt( d / (2 - tr1) );

      if E(1,3) > E(3,1)
        v(2) = -v(2);
      end

      v(1) = (E(1,2) + E(2,1)) * v(2) / d;
      v(3) = (E(2,3) + E(3,2)) * v(2) / d;

    else

      d = 2 * E(3,3) - tr1;
      v(3) = S * sqrt(d / (2 - tr1));

      if E(2,1) > E(1,2)
        v(3) = -v(3);
      end

      v(1) = (E(1,3) + E(3,1)) * v(3) / d;
      v(2) = (E(2,3) + E(3,2)) * v(3) / d;

    end

  else

    S = sqrt(S);
    S = atan2(S, tr1) / S;

    v = v * S;
  end
