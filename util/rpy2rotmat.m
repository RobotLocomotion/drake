function [rotMat,drotmat,ddrotmat] = rpy2rotmat(rpy)
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
if(nargout>2)
  rr_idx = 1;
  rp_idx = 2;
  ry_idx = 3;
  pr_idx = 4;
  pp_idx = 5;
  py_idx = 6;
  yr_idx = 7;
  yp_idx = 8;
  yy_idx = 9;
  
  ddR_row = zeros(53,1);
  ddR_col = zeros(53,1);
  ddR_val = zeros(53,1);
  ddR_nnz = 1;
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(1,yy_idx,-cos_y*cos_p,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(1,yp_idx,sin_y*sin_p,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(1,py_idx,sin_y*sin_p,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(1,pp_idx,-cos_y*cos_p,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(2,yy_idx,-sin_y*cos_p,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(2,yp_idx,-cos_y*sin_p,ddR_row,ddR_col,ddR_val,ddR_nnz);

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(2,py_idx,-cos_y*sin_p,ddR_row,ddR_col,ddR_val,ddR_nnz);

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(2,pp_idx,-sin_y*cos_p,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(3,pp_idx,sin_p,ddR_row,ddR_col,ddR_val,ddR_nnz);

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,rr_idx,-cos_y*sin_p*sin_r+sin_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,rp_idx,cos_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,ry_idx,-sin_y*sin_p*cos_r+cos_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,pr_idx,cos_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,pp_idx,-cos_y*sin_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,py_idx,-sin_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,yr_idx,-sin_y*sin_p*cos_r+cos_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);  

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,yp_idx,-sin_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);  

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(4,yy_idx,-cos_y*sin_p*sin_r+sin_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,rr_idx,-sin_y*sin_p*sin_r-cos_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,rp_idx,sin_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);  

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,ry_idx,cos_y*sin_p*cos_r+sin_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,pr_idx,sin_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
 
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,pp_idx,-sin_y*sin_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz); 

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,py_idx,cos_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);  

  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,yr_idx,cos_y*sin_p*cos_r+sin_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,yp_idx,cos_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);  
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(5,yy_idx,-sin_y*sin_p*sin_r-cos_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(6,rr_idx,-cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(6,rp_idx,-sin_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(6,pr_idx,-sin_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(6,pp_idx,-cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,rr_idx,-cos_y*sin_p*cos_r-sin_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,rp_idx,-cos_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,ry_idx,sin_y*sin_p*sin_r+cos_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,pr_idx,-cos_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,pp_idx, -cos_y*sin_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,py_idx, -sin_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,yr_idx,sin_y*sin_p*sin_r+cos_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,yp_idx, -sin_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(7,yy_idx,-cos_y*sin_p*cos_r-sin_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,rr_idx,-sin_y*sin_p*cos_r+cos_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,rp_idx,-sin_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,ry_idx,-cos_y*sin_p*sin_r+sin_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,pr_idx,-sin_y*cos_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,pp_idx, -sin_y*sin_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,py_idx,cos_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,yr_idx, -cos_y*sin_p*sin_r+sin_y*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,yp_idx,cos_y*cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(8,yy_idx,-sin_y*sin_p*cos_r+cos_y*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(9,rr_idx, -cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(9,rp_idx,sin_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(9,pr_idx,sin_p*sin_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(9,pp_idx,-cos_p*cos_r,ddR_row,ddR_col,ddR_val,ddR_nnz);
  
  ddrotmat = sparse(ddR_row(1:ddR_nnz-1),ddR_col(1:ddR_nnz-1),ddR_val(1:ddR_nnz-1),9,9);
end
end

function [ddR_row,ddR_col,ddR_val,ddR_nnz] = addEntry_ddR(row,col,val,ddR_row,ddR_col,ddR_val,ddR_nnz)
ddR_row(ddR_nnz) = row;
ddR_col(ddR_nnz) = col;
ddR_val(ddR_nnz) = val;
ddR_nnz = ddR_nnz+1;
end
