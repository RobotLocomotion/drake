classdef CableLength < drakeFunction.kinematic.Kinematic
% Describes a cable wound around pulleys as derived in THEORY OF MACHINES
% by V.RAVI - see Github Issue #546 for more details.

  methods
    function obj = CableLength(rbm,name)
      obj = obj@drakeFunction.kinematic.Kinematic(rbm,1);
      obj.name = name;
    end

    function obj = addPulley(obj,frame,xyz,axis,radius,number_of_wraps)
      % note: the order of addition DOES MATTER

      p.frame = frame;
      p.xyz = xyz;
      p.axis = axis;
      p.radius = radius;
      p.number_of_wraps = number_of_wraps;
      
      obj.pulley = horzcat(obj.pulley, p);
    end
    
    function [length,dlength,ddlength] = eval(obj,q)
      kinsol = obj.rbm.doKinematics(q,nargout>2);

      length = 0;
      dlength = 0*q';
      ddlength = zeros(1,numel(q)^2); % unused variable
      for i=1:numel(obj.pulley)
        % compute new positions of each pulley (?)
        if nargout>2
          [pt,dpt,ddpt] = forwardKin(obj.rbm,kinsol,obj.pulley(i).frame,obj.pulley(i).xyz);
        elseif nargout>1
          [pt,dpt] = forwardKin(obj.rbm,kinsol,obj.pulley(i).frame,obj.pulley(i).xyz);
        else
          pt = forwardKin(obj.rbm,kinsol,obj.pulley(i).frame,obj.pulley(i).xyz);
        end          
        
        length = length + 2*pi*obj.pulley(i).radius*obj.pulley(i).number_of_wraps;

        % for the first pulley, just initialize the pulley location values (last_pt and last_dpt)
        % then do math on the cable between the current and last pulley
        if i>1
          vec = pt-last_pt; % last_pt is the pt from the previous pulley
          C = sqrt(vec'*vec); % abs distance between the two pulleys
          %Csq = vec'*vec; %squared distance between the two pulleys
          
          if nargout>1
            dvec = dpt-last_dpt;
            dC = vec'*dvec/C;
            
            if nargout>2
              ddvec = ddpt - last_ddpt;
              ddC = matGradMultMat(vec',dvec,dvec',reshape(ddvec,9,[]))/C - (vec'*dvec)'/C^2*dC;
%               ddC = (dvec'*dvec + vec'*ddvec)/C - vec'*dvec*dC/(C^2);
            end
          end
          
          r1 = obj.pulley(i-1).radius;
          r2 = obj.pulley(i).radius;
          % if the pulleys overlap, skip this case
          if (C<r1+r2+eps) % cut me a little slack, eh?
            continue;  % TODO: just skip this one... because the optimizers might actually get here
          end
          if r1>0 || r2>0 % at least one pulley is a physical object
            alignment = dot(obj.pulley(i-1).axis,obj.pulley(i).axis);
            if r1>0 && r2>0 % both pulleys are physical objects, so make sure the axes are aligned
              assert(abs(alignment)-1>-1e-8,'Drake:CablesAndPulleys:AxisAlignedPulleys','Neighboring pulleys with radius>0 must be axis-aligned.  Consider adding a radius zero pulley if you need to "bend" around a corner.');
            end
            
            % as described in https://github.com/RobotLocomotion/drake/issues/546
            cvec = vec/C; % unit vector of line from prev pulley to curr pulley
            
            if alignment>0 % both pulleys rotate in same direction
              % then it's like an open flat belt drive
              % see image here:
              % https://cloud.githubusercontent.com/assets/6442292/4991473/3dcb877c-6962-11e4-86e5-5229290a3526.png
              s = (r2-r1)/C;  
              alpha = asin(s); % contact angle of belt
              % contact pts of belt to pulley 1 and 2
              pt1 = last_pt + r1*axis2rotmat([obj.pulley(i-1).axis;-pi/2-alpha])*cvec;
              pt2 = pt + r2*axis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*cvec;
              if nargout>1
                ds = -(r2-r1)/C^2*dC;
                dalpha = 1/sqrt(1-s^2)*ds;
                dcvec = dvec/C - vec/C^2*dC;
                if r1>0
                  dpt1 = last_dpt - r1*daxis2rotmatdtheta([obj.pulley(i-1).axis;-pi/2-alpha])*cvec*dalpha + r1*axis2rotmat([obj.pulley(i-1).axis;-pi/2-alpha])*dcvec;  
                else % it acts as a corner; seems like above expression would still be valid unless negative radii mean something else
                  dpt1 = last_dpt; 
                end
                if r2>0
                  dpt2 = dpt - r2*daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha])*cvec*dalpha + r2*axis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*dcvec;           
                else
                  dpt2 = dpt;
                end
                if nargout>2
                  dds = 2/C^3*(r2-r1)*(dC'*dC) - (r2-r1)/C^2*ddC;
                  ddalpha = s/(1-s^2)^(3/2)*ds^2 + dds/sqrt(1-s^2);
                  ddcvec = ddvec/C - 2*reshape([dvec*dC(1); dvec*dC(2); dvec*dC(3)],3,[]) + 2*vec/C^3*dC^2 - v/C^2*ddC;
                  if r1>0
                    ddpt1 = last_ddpt + r1*ddaxis2rotmatdtheta([obj.pulley(i-1).axis;-pi/2-alpha])*cvec*dalpha^2 - r1*daxis2rotmatdtheta([obj.pulley(i-1).axis;-pi/2-alpha])*cvec*ddalpha - r1*daxis2rotmatdtheta([obj.pulley(i-1).axis;-pi/2-alpha])*dcvec*dalpha - ...
                      r1*daxis2rotmatdtheta([obj.pulley(i-1).axis;-pi/2-alpha])*dcvec*dalpha + r1*axis2rotmat([obj.pulley(i-1).axis;-pi/2-alpha])*ddcvec;
                  else % it acts as a corner; seems like above expression would still be valid unless negative radii mean something else
                    ddpt1 = last_ddpt;
                  end
                  if r2>0
                    ddpt2 = ddpt + r2*ddaxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha])*cvec*dalpha^2 - r2*daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha])*cvec*ddalpha - r2*daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha])*dcvec*dalpha - ...
                      r2*daxis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*dcvec*dalpha + r2*axis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*ddcvec;
                  else
                    ddpt2 = ddpt;
                  end
                end
              end
            
            else % the pulleys rotate in opposite directions, then it's like a cross flat belt drive
              % see image here:
              % https://cloud.githubusercontent.com/assets/6442292/4991474/3dcc88f2-6962-11e4-952d-8385566e4f6c.png
              s = (r1+r2)/C;
              alpha = asin(s); % contact angle of belt
              % contact pts of belt to pulley 1 and 2
              pt1 = last_pt + r1*axis2rotmat([obj.pulley(i-1).axis;-pi/2+alpha])*cvec;
              pt2 = pt + r2*axis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*cvec;

              if nargout>1
                ds = -(r1+r2)/C^2*dC;
                dalpha = 1/sqrt(1-s^2)*ds;
                dcvec = dvec/C - vec/C^2*dC;
                if r1>0
                  dpt1 = last_dpt + r1*daxis2rotmatdtheta([obj.pulley(i-1).axis;-pi/2+alpha])*cvec*dalpha + r1*axis2rotmat([obj.pulley(i-1).axis;-pi/2+alpha])*dcvec;
                else
                  dpt1 = last_dpt;
                end
                if r2>0
                  dpt2 = dpt - r2*daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha])*cvec*dalpha + r2*axis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*dcvec;
                else
                  dpt2 = dpt;
                end
                
                if nargout>2
                  dds = 2/C^3*(r1+r2)*(dC'*dC) - (r1+r2)/C^2*ddC;
                  ddalpha = s/(1-s^2)^(3/2)*(ds'*ds) + dds/sqrt(1-s^2);
                  ddcvec = ddvec/C - 1/C^2*kron(dvec,dC) + reshape(matGradMultMat(vec,dC,dvec,ddC)/C^2, 3, []);
                  if r1>0
                    ddpt1 = last_ddpt + r1*reshape(matGradMultMat(daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2+alpha])*cvec, dalpha, ddaxis2rotmatdtheta([obj.pulley(i).axis;-pi/2+alpha])*cvec*dalpha, ddalpha) + ...
                      r1*matGradMultMat(axis2rotmat([obj.pulley(i).axis;-pi/2+alpha]),dcvec, kron(daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2+alpha]),dalpha'), reshape(ddcvec,9,[])), 3, []);
                  else
                    ddpt1 = last_ddpt;
                  end
                  if r2>0
                    ddpt2 = ddpt - r2*reshape(matGradMultMat(daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha])*cvec, dalpha, ddaxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha])*cvec*dalpha, ddalpha) + ...
                      r2*matGradMultMat(axis2rotmat([obj.pulley(i).axis;-pi/2-alpha]),dcvec, kron(daxis2rotmatdtheta([obj.pulley(i).axis;-pi/2-alpha]),dalpha'), reshape(ddcvec,9,[])), 3, []);
                  else
                    ddpt2 = ddpt;
                  end
                  
                end
              end
              
            end
                        
            vec = pt2-pt1; % belt between pulleys (aka length not in contact with pulley)
            C = sqrt(vec'*vec);
            length = length+C;
            
            if nargout>1
              dvec = dpt2 - dpt1;
              dC = vec'*dvec/(C+eps);
              dlength = dlength+dC;
              if nargout>2
                ddvec = ddpt2 - ddpt1;
                ddC = matGradMultMat(vec',dvec,dvec',reshape(ddvec,9,[]))/(C+eps) - (vec'*dvec)'/(C+eps)^2*dC;
                ddlength = ddlength + reshape(ddC, 1, []);
              end
            end
                        
            if r1>0 % now add in the arc length between pt1 and last_attachment_pt (length of pulley-cable contact)
              % unit vectors pointing from center to belt leaving/entering contact points
              v1 = (pt1-last_pt)/r1; v2 = (last_attachment_pt-last_pt)/r1;
              c = dot(v1,v2); svec = cross(v1,v2); s = sqrt(svec'*svec);
              theta = atan2(s,c); % contact arc angle
              if theta<0, theta=theta+2*pi; end
              length = length+theta*r1;
              
              if nargout>1
                dv1 = (dpt1-last_dpt)/r1; dv2 = (last_attachment_dpt-last_dpt)/r1;
                dc = v2'*dv1+v1'*dv2; dsvec=dcross(v1,v2,dv1,dv2); ds = svec'*dsvec/max(s,eps);
                dtheta = -s*dc + c*ds;
                dlength = dlength + dtheta*r1;
                if nargout>2
                  ddv1 = (ddpt1 - last_ddpt)/r1; ddv2 = (last_attachment_ddpt-last_ddpt)/r1;
                  ddc = matGradMultMat(v2',dv1,dv2',reshape(ddv1,9,[])) + matGradMultMat(v1',dv2,dv1',reshape(ddv2,9,[])); 
                  ddsvec = ddcross(v1,v2,dv1,dv2,ddv1,ddv2);
                  dds = -(svec'*dsvec)'*(svec'*dsvec)/max(eps,(svec'*svec)^(3/2)) + matGradMultMat(svec',dsvec,dsvec',reshape(ddsvec,9,[]))/max(s,eps);
                  ddtheta = -ds'*dc - s*ddc + dc'*ds + c*dds;
                  ddlength = ddlength + reshape(ddtheta,1,[])*r1;
                end
              end
            end
            last_attachment_pt = pt2;
              
            if nargout>1
              last_attachment_dpt = dpt2;
              if nargout>2
                last_attachment_ddpt = ddpt2;
              end
            end
          else % both pulleys are actually corners used to bend cable in 3D
            length = length+C; % C is dist between the two bending pts
            last_attachment_pt = pt;
            if nargout>1
              dlength = dlength+dC;
              last_attachment_dpt = dpt;
              if nargout>2
                ddlength = ddlength + ddC;
                last_attachment_ddpt = ddpt;
              end
            end
          end
        else % initialize values for the first pulley
          last_attachment_pt = pt;
          if nargout>1
            last_attachment_dpt = dpt;
            if nargout>2
              last_attachment_ddpt = ddpt;
            end
          end
        end
        
        last_pt = pt; 
        if nargout>1
          last_dpt = dpt;
          if nargout>2
            last_ddpt = ddpt;
          end
        end
      end
      
    end
    
    function [vertex,edge] = getSegments(obj,q)
      % this method is intended to make is easy to display the cable
      % @retval vertices is a 3-by-n list of 3D points (in world
      % coordinates)
      % @retval edges is a 2-by-m list of integer indices into vertices -
      % one column for each line segment
      % Note: we use this format because there is not necessarily a line
      % that should be drawn between each sequential vertex (e.g. it would
      % draw a line directly through a pulley).  
      
      kinsol = obj.rbm.doKinematics(q);

      vertex = [];
      edge = [];
      for i=1:numel(obj.pulley)
        pt = forwardKin(obj.rbm,kinsol,obj.pulley(i).frame,obj.pulley(i).xyz);
        
        if i>1
          r1 = obj.pulley(i-1).radius;
          r2 = obj.pulley(i).radius;
          if r1>0 || r2>0
            alignment = dot(obj.pulley(i-1).axis,obj.pulley(i).axis);
            if r1>0 && r2>0 % then make sure the axes are aligned
              assert(abs(alignment)-1>-1e-8,'Drake:CablesAndPulleys:AxisAlignedPulleys','Neighboring pulleys with radius>0 must be axis-aligned.  Consider adding a radius zero pulley if you need to "bend" around a corner.');
            end
            
            % as described in https://github.com/RobotLocomotion/drake/issues/546
            vec = pt-last_pt;
            C = sqrt(vec'*vec);
            cvec = vec/C;
            
            if alignment>0 % then it's like an open flat belt drive
              alpha = asin((r2-r1)/C);
              pt1 = last_pt + r1*axis2rotmat([obj.pulley(i-1).axis;-pi/2-alpha])*cvec;
              pt2 = pt + r2*axis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*cvec;
            else % then it's like a cross flat belt drive
              alpha = asin((r1+r2)/C);
              pt1 = last_pt + r1*axis2rotmat([obj.pulley(i-1).axis;-pi/2+alpha])*cvec;
              pt2 = pt + r2*axis2rotmat([obj.pulley(i).axis;-pi/2-alpha])*cvec;
            end
              
            
            vertex = horzcat(vertex,[pt1,pt2]);
            n = size(vertex,2);
            edge = horzcat(edge,[n-1;n]);
          else
            vertex = horzcat(vertex,[last_pt,pt]);
            n = size(vertex,2);
            edge = horzcat(edge,[n-1;n]);
          end
        end
        
        last_pt = pt; 
      end
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      for i=1:numel(obj.pulley)
        obj.pulley(i).frame = map_from_old_to_new(obj.pulley(i).frame);
      end
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      for i=1:numel(obj.pulley)
        if (obj.pulley(i).frame == body_ind)
          obj.pulley(i).xyz = model.body(body_ind).Ttree(1:end-1,:)*[obj.pulley(i).xyz;1];
          obj.pulley(i).frame = model.body(body_ind).parent;
        end
      end
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      error('need to implement this.  (see changeRootLink)');
    end
    
  end

  properties
    pulley = [];
  end
end
