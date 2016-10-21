function h = drawPlane(plane_normal,plane_pt,plane_size,lcmgl_struct)
% draw a square plane in lcmgl or matlab viewer in 3D
% @param plane_normal   A unit vector, the normal of the plane
% @param plane_pt       A 3 x 1 vector. A point on the plane. The plane is
% drawn around plane_pt
% @param plane_size           A scalar, a rectangle of dimension size x size will
% be drawn around plane_pt
% @param lcmgl_struct      -- use_lcmgl  A boolean, true if drawn in lcmgl, false if in
% MATLAB visualizer
%                          -- lcmgl_name  A string of the lcmgl object
% @retval h      The handle of the plot, if it is plot in MATLAB
if(any(size(plane_normal)~=[3,1]))
  error('plane_normal should be a 3 x 1 vector');
end
norm_plane_normal = norm(plane_normal);
if(norm_plane_normal<1e-5)
  error('plane_normal should be a non-zero vector');
end
plane_normal = plane_normal/norm_plane_normal;
if(any(size(plane_pt)~=[3,1]))
  error('plane_pt should be a 3 x 1 vector');
end
if(numel(plane_size)~=1 || plane_size<=0)
  error('plane_size should be a positive scalar');
end
if(~isstruct(lcmgl_struct) || ~isfield(lcmgl_struct,'use_lcmgl') || ~isfield(lcmgl_struct,'lcmgl_name'))
  error('lcmgl_struct should have field use_lcmgl and lcmgl_name');
end
if(lcmgl_struct.use_lcmgl)
  checkDependency('lcmgl');
  h = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,lcmgl_struct.lcmgl_name);
  h.glColor4f(0, 0, 1, 0.5)
  h.glPushMatrix();
  h.glTranslated(plane_pt(1),plane_pt(2),plane_pt(3));
  rotate_angle_axis = rotmat2axis(rotateVectorToAlign(plane_normal,[0;0;1]));
  h.glRotated(-rotate_angle_axis(4)/pi*180,rotate_angle_axis(1),rotate_angle_axis(2),rotate_angle_axis(3));  
  h.box([0;0;0],[plane_size/2;plane_size/2;0.001]);
  h.glPopMatrix();
else
  plane_vert0 = plane_size/2*[1 -1 -1 1;1 1 -1 -1;0 0 0 0];
  plane_vert = rotateVectorToAlign(plane_normal,[0;0;1])*plane_vert0+bsxfun(@times,plane_pt,ones(1,4));
  h = fill3(plane_vert(1,:),plane_vert(2,:),plane_vert(3,:),[0 0 1]);
end
end
