function [rotMat,drotmat] = rpy2rotmat(rpy)
% equivalent to rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1))

rotMat = [cos(rpy(3))*cos(rpy(2)) cos(rpy(3))*sin(rpy(2))*sin(rpy(1))-sin(rpy(3))*cos(rpy(1)) cos(rpy(3))*sin(rpy(2))*cos(rpy(1))+sin(rpy(3))*sin(rpy(1));...
    sin(rpy(3))*cos(rpy(2)) sin(rpy(3))*sin(rpy(2))*sin(rpy(1))+cos(rpy(3))*cos(rpy(1)) sin(rpy(3))*sin(rpy(2))*cos(rpy(1))-cos(rpy(3))*sin(rpy(1));...
    -sin(rpy(2)) cos(rpy(2))*sin(rpy(1)) cos(rpy(2))*cos(rpy(1))];
  if(nargout>1)
    drotmat_row = zeros(21,1);
    drotmat_col = zeros(21,1);
    drotmat_val = zeros(21,1);
    drotmat_row(1) = 1;
    drotmat_col(1) = 2;
    drotmat_val(1) = cos(rpy(3))*-sin(rpy(2));
    
    drotmat_row(2) = 1;
    drotmat_col(2) = 3;
    drotmat_val(2) = -sin(rpy(3))*cos(rpy(2));
    
    drotmat_row(3) = 2;
    drotmat_col(3) = 2;
    drotmat_val(3) = sin(rpy(3))*-sin(rpy(2));
    
    drotmat_row(4) = 2;
    drotmat_col(4) = 3;
    drotmat_val(4) = cos(rpy(3))*cos(rpy(2));
    
    drotmat_row(5) = 3;
    drotmat_col(5) = 2;
    drotmat_val(5) = -cos(rpy(2));
    
    drotmat_row(6) = 4;
    drotmat_col(6) = 1;
    drotmat_val(6) = cos(rpy(3))*sin(rpy(2))*cos(rpy(1))+sin(rpy(3))*sin(rpy(1));
    
    drotmat_row(7) = 4;
    drotmat_col(7) = 2;
    drotmat_val(7) = cos(rpy(3))*cos(rpy(2))*sin(rpy(1));
    
    drotmat_row(8) = 4;
    drotmat_col(8) = 3;
    drotmat_val(8) = -sin(rpy(3))*sin(rpy(2))*sin(rpy(1))-cos(rpy(3))*cos(rpy(1));
    
    drotmat_row(9) = 5;
    drotmat_col(9) = 1;
    drotmat_val(9) = sin(rpy(3))*sin(rpy(2))*cos(rpy(1))-cos(rpy(3))*sin(rpy(1));
    
    drotmat_row(10) = 5;
    drotmat_col(10) = 2;
    drotmat_val(10) = sin(rpy(3))*cos(rpy(2))*sin(rpy(1));
    
    drotmat_row(11) = 5;
    drotmat_col(11) = 3;
    drotmat_val(11) = cos(rpy(3))*sin(rpy(2))*sin(rpy(1))-sin(rpy(3))*cos(rpy(1));
    
    drotmat_row(12) = 6;
    drotmat_col(12) = 1;
    drotmat_val(12) = cos(rpy(2))*cos(rpy(1));
    
    drotmat_row(13) = 6;
    drotmat_col(13) = 2;
    drotmat_val(13) = -sin(rpy(2))*sin(rpy(1));
    
    drotmat_row(14) = 7;
    drotmat_col(14) = 1;
    drotmat_val(14) = cos(rpy(3))*sin(rpy(2))*-sin(rpy(1))+sin(rpy(3))*cos(rpy(1));
    
    drotmat_row(15) = 7;
    drotmat_col(15) = 2;
    drotmat_val(15) = cos(rpy(3))*cos(rpy(2))*cos(rpy(1));
    
    drotmat_row(16) = 7;
    drotmat_col(16) = 3;
    drotmat_val(16) = -sin(rpy(3))*sin(rpy(2))*cos(rpy(1))+cos(rpy(3))*sin(rpy(1));
    
    drotmat_row(17) = 8;
    drotmat_col(17) = 1;
    drotmat_val(17) = sin(rpy(3))*sin(rpy(2))*-sin(rpy(1))-cos(rpy(3))*cos(rpy(1));
    
    drotmat_row(18) = 8;
    drotmat_col(18) = 2;
    drotmat_val(18) = sin(rpy(3))*cos(rpy(2))*cos(rpy(1));
    
    drotmat_row(19) = 8;
    drotmat_col(19) = 3;
    drotmat_val(19) = cos(rpy(3))*sin(rpy(2))*cos(rpy(1))+sin(rpy(3))*sin(rpy(1));
     
    drotmat_row(20) = 9;
    drotmat_col(20) = 1;
    drotmat_val(20) = -cos(rpy(2))*sin(rpy(1));
    
    drotmat_row(21) = 9;
    drotmat_col(21) = 2;
    drotmat_val(21) = -sin(rpy(2))*cos(rpy(1));
    drotmat = sparse(drotmat_row,drotmat_col,drotmat_val,9,3);
  end
end