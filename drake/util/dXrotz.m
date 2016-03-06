function  dX = dXrotz( theta )
% derivative of featherstone's Xrotz

dc = -sin(theta);  %c = cos(theta);
ds = cos(theta); % s = sin(theta);

dX = [  dc  ds  0  0  0  0 ;
      -ds  dc  0  0  0  0 ;
       0  0  0  0  0  0 ;
       0  0  0  dc  ds  0 ;
       0  0  0 -ds  dc  0 ;
       0  0  0  0  0  0
    ];
