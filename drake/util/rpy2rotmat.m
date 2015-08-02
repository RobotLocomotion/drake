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
  drotmat = sparse(9,3);
  
  drotmat(1,2) = cos_y*-sin_p;
  
  drotmat(1,3) = -sin_y*cos_p;
    
  drotmat(2,2) = sin_y*-sin_p;
    
  drotmat(2,3) = cos_y*cos_p;
    
  drotmat(3,2) = -cos_p;
    
  drotmat(4,1) = cos_y*sin_p*cos_r+sin_y*sin_r;
    
  drotmat(4,2) = cos_y*cos_p*sin_r;
    
  drotmat(4,3) = -sin_y*sin_p*sin_r-cos_y*cos_r;
    
  drotmat(5,1) = sin_y*sin_p*cos_r-cos_y*sin_r;
    
  drotmat(5,2) = sin_y*cos_p*sin_r;
    
  drotmat(5,3) = cos_y*sin_p*sin_r-sin_y*cos_r;
    
  drotmat(6,1) = cos_p*cos_r;
    
  drotmat(6,2) = -sin_p*sin_r;
    
  drotmat(7,1) = cos_y*sin_p*-sin_r+sin_y*cos_r;
    
  drotmat(7,2) = cos_y*cos_p*cos_r;
    
  drotmat(7,3) = -sin_y*sin_p*cos_r+cos_y*sin_r;
    
  drotmat(8,1) = sin_y*sin_p*-sin_r-cos_y*cos_r;
    
  drotmat(8,2) = sin_y*cos_p*cos_r;
    
  drotmat(8,3) = cos_y*sin_p*cos_r+sin_y*sin_r;
     
  drotmat(9,1) = -cos_p*sin_r;
    
  drotmat(9,2) = -sin_p*cos_r;
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
  
  ddrotmat = sparse(9,9);
  
  ddrotmat(1,yy_idx) = -cos_y*cos_p;
  
  ddrotmat(1,yp_idx) = sin_y*sin_p;
  
  ddrotmat(1,pp_idx) = -cos_y*cos_p;
  
  ddrotmat(1,py_idx) = sin_y*sin_p;
  
  ddrotmat(2,yy_idx) = -sin_y*cos_p;  
  
  ddrotmat(2,yp_idx) = -cos_y*sin_p;
  
  ddrotmat(2,py_idx) = -cos_y*sin_p;
  
  ddrotmat(2,pp_idx) = -sin_y*cos_p;  
  
  ddrotmat(3,pp_idx) = sin_p;
  
  ddrotmat(4,rr_idx) = -cos_y*sin_p*sin_r+sin_y*cos_r;
  
  ddrotmat(4,rp_idx) = cos_y*cos_p*cos_r;
  
  ddrotmat(4,ry_idx) = -sin_y*sin_p*cos_r+cos_y*sin_r;
  
  ddrotmat(4,pr_idx) = cos_y*cos_p*cos_r;
  
  ddrotmat(4,pp_idx) = -cos_y*sin_p*sin_r;
  
  ddrotmat(4,py_idx) = -sin_y*cos_p*sin_r;

  ddrotmat(4,yr_idx) = -sin_y*sin_p*cos_r+cos_y*sin_r;  

  ddrotmat(4,yp_idx) = -sin_y*cos_p*sin_r;  

  ddrotmat(4,yy_idx) = -cos_y*sin_p*sin_r+sin_y*cos_r;
  
  ddrotmat(5,rr_idx) = -sin_y*sin_p*sin_r-cos_y*cos_r;

  ddrotmat(5,rp_idx) = sin_y*cos_p*cos_r;  

  ddrotmat(5,ry_idx) = cos_y*sin_p*cos_r+sin_y*sin_r;
  
  ddrotmat(5,pr_idx) = sin_y*cos_p*cos_r;
 
  ddrotmat(5,pp_idx) = -sin_y*sin_p*sin_r; 

  ddrotmat(5,py_idx) = cos_y*cos_p*sin_r;  

  ddrotmat(5,yr_idx) = cos_y*sin_p*cos_r+sin_y*sin_r;
  
  ddrotmat(5,yp_idx) = cos_y*cos_p*sin_r;  
  
  ddrotmat(5,yy_idx) = -sin_y*sin_p*sin_r-cos_y*cos_r;
  
  ddrotmat(6,rr_idx) = -cos_p*sin_r;
  
  ddrotmat(6,rp_idx) = -sin_p*cos_r;
  
  ddrotmat(6,pr_idx) = -sin_p*cos_r;
  
  ddrotmat(6,pp_idx) = -cos_p*sin_r;
  
  ddrotmat(7,rr_idx) = -cos_y*sin_p*cos_r-sin_y*sin_r;
  
  ddrotmat(7,rp_idx) = -cos_y*cos_p*sin_r;
  
  ddrotmat(7,ry_idx) = sin_y*sin_p*sin_r+cos_y*cos_r;
  
  ddrotmat(7,pr_idx) = -cos_y*cos_p*sin_r;
  
  ddrotmat(7,pp_idx) =  -cos_y*sin_p*cos_r;
  
  ddrotmat(7,py_idx) =  -sin_y*cos_p*cos_r;
  
  ddrotmat(7,yr_idx) = sin_y*sin_p*sin_r+cos_y*cos_r;
  
  ddrotmat(7,yp_idx) =  -sin_y*cos_p*cos_r;
  
  ddrotmat(7,yy_idx) = -cos_y*sin_p*cos_r-sin_y*sin_r;
  
  ddrotmat(8,rr_idx) = -sin_y*sin_p*cos_r+cos_y*sin_r;
  
  ddrotmat(8,rp_idx) = -sin_y*cos_p*sin_r;
  
  ddrotmat(8,ry_idx) = -cos_y*sin_p*sin_r+sin_y*cos_r;
  
  ddrotmat(8,pr_idx) = -sin_y*cos_p*sin_r;
  
  ddrotmat(8,pp_idx) =  -sin_y*sin_p*cos_r;
  
  ddrotmat(8,py_idx) = cos_y*cos_p*cos_r;
  
  ddrotmat(8,yr_idx) =  -cos_y*sin_p*sin_r+sin_y*cos_r;
  
  ddrotmat(8,yp_idx) = cos_y*cos_p*cos_r;
  
  ddrotmat(8,yy_idx) = -sin_y*sin_p*cos_r+cos_y*sin_r;
  
  ddrotmat(9,rr_idx) =  -cos_p*cos_r;
  
  ddrotmat(9,rp_idx) = sin_p*sin_r;
  
  ddrotmat(9,pr_idx) = sin_p*sin_r;
  
  ddrotmat(9,pp_idx) = -cos_p*cos_r;
  
end
end

