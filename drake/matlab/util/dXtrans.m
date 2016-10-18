function  dX = dXtrans( r )

% derivative of featherstone's Xtrans function

dXr1 = [0     0     0    0  0  0 ;
        0     0     0    0  0  0 ;
        0     0     0    0  0  0 ;
        0     0     0    0  0  0 ;
        0     0     1    0  0  0 ;
        0    -1     0    0  0  0 ];
        
dXr2 = [0     0     0    0  0  0 ;
        0     0     0    0  0  0 ;
        0     0     0    0  0  0 ;
        0     0    -1    0  0  0 ;
        0     0     0    0  0  0 ;
        1     0     0    0  0  0 ];
        
dXr3 = [0     0     0    0  0  0 ;
        0     0     0    0  0  0 ;
        0     0     0    0  0  0 ;
        0     1     0    0  0  0 ;
       -1     0     0    0  0  0 ;
        0 	  0     0    0  0  0 ];

dX = reshape([dXr1 dXr2 dXr3],36,3);
    
end