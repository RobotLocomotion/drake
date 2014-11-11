function [rotMat,drotmat] = rpy2rotmat(rpy)
% equivalent to rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1))

cos_r = cos(rpy(1));
sin_r = sin(rpy(1));
cos_p = cos(rpy(2));
sin_p = sin(rpy(2));
cos_y = cos(rpy(3));
sin_y = sin(rpy(3));
rotMat = [cos_y*cos_p cos_y*sin_p*sin_r-sin_y*cos_r cos_y*sin_p*cos_r+sin_y*sin_r;...
    sin_y*cos_p sin_y*sin_p*sin_r+cos_y*cos_r sin_y*sin_p*cos_r-cos_y*sin_r;...
    -sin_p cos_p*sin_r cos_p*cos_r];
if(nargout>1)
  drotmat_row = zeros(21,1);
  drotmat_col = zeros(21,1);
  drotmat_val = zeros(21,1);
  drotmat_row(1) = 1;
  drotmat_col(1) = 2;
  drotmat_val(1) = cos_y*-sin_p;
  
  drotmat_row(2) = 1;
  drotmat_col(2) = 3;
  drotmat_val(2) = -sin_y*cos_p;
    
  drotmat_row(3) = 2;
  drotmat_col(3) = 2;
  drotmat_val(3) = sin_y*-sin_p;
    
  drotmat_row(4) = 2;
  drotmat_col(4) = 3;
  drotmat_val(4) = cos_y*cos_p;
    
  drotmat_row(5) = 3;
  drotmat_col(5) = 2;
  drotmat_val(5) = -cos_p;
    
  drotmat_row(6) = 4;
  drotmat_col(6) = 1;
  drotmat_val(6) = cos_y*sin_p*cos_r+sin_y*sin_r;
    
  drotmat_row(7) = 4;
  drotmat_col(7) = 2;
  drotmat_val(7) = cos_y*cos_p*sin_r;
    
  drotmat_row(8) = 4;
  drotmat_col(8) = 3;
  drotmat_val(8) = -sin_y*sin_p*sin_r-cos_y*cos_r;
    
  drotmat_row(9) = 5;
  drotmat_col(9) = 1;
  drotmat_val(9) = sin_y*sin_p*cos_r-cos_y*sin_r;
    
  drotmat_row(10) = 5;
  drotmat_col(10) = 2;
  drotmat_val(10) = sin_y*cos_p*sin_r;
    
  drotmat_row(11) = 5;
  drotmat_col(11) = 3;
  drotmat_val(11) = cos_y*sin_p*sin_r-sin_y*cos_r;
    
  drotmat_row(12) = 6;
  drotmat_col(12) = 1;
  drotmat_val(12) = cos_p*cos_r;
    
  drotmat_row(13) = 6;
  drotmat_col(13) = 2;
  drotmat_val(13) = -sin_p*sin_r;
    
  drotmat_row(14) = 7;
  drotmat_col(14) = 1;
  drotmat_val(14) = cos_y*sin_p*-sin_r+sin_y*cos_r;
    
  drotmat_row(15) = 7;
  drotmat_col(15) = 2;
  drotmat_val(15) = cos_y*cos_p*cos_r;
    
  drotmat_row(16) = 7;
  drotmat_col(16) = 3;
  drotmat_val(16) = -sin_y*sin_p*cos_r+cos_y*sin_r;
    
  drotmat_row(17) = 8;
  drotmat_col(17) = 1;
  drotmat_val(17) = sin_y*sin_p*-sin_r-cos_y*cos_r;
    
  drotmat_row(18) = 8;
  drotmat_col(18) = 2;
  drotmat_val(18) = sin_y*cos_p*cos_r;
    
  drotmat_row(19) = 8;
  drotmat_col(19) = 3;
  drotmat_val(19) = cos_y*sin_p*cos_r+sin_y*sin_r;
     
  drotmat_row(20) = 9;
  drotmat_col(20) = 1;
  drotmat_val(20) = -cos_p*sin_r;
    
  drotmat_row(21) = 9;
  drotmat_col(21) = 2;
  drotmat_val(21) = -sin_p*cos_r;
  drotmat = sparse(drotmat_row,drotmat_col,drotmat_val,9,3);
end
end
