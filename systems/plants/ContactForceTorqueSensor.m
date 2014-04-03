classdef ContactForceTorqueSensor < TimeSteppingRigidBodySensorWithState %& Visualizer

 properties
   normal_ind=[];
   tangent_ind=[];
   planar_inds=[];
   kinframe;
%   manip;  % warning: could get stale, but keeping it around for the draw method
 end

 methods
   function obj = ContactForceTorqueSensor(tsmanip,frame)
     % default frame to initialize visualizer
%      if tsmanip.twoD
%        fr = CoordinateFrame('DefaultForceTorqueFrame',getNumStates(tsmanip)+3,'f');
%      else
%        fr = CoordinateFrame('DefaultForceTorqueFrame',getNumStates(tsmanip)+6,'f');
%      end
%      obj = obj@Visualizer(fr);

     typecheck(frame,'RigidBodyFrame');
     obj.kinframe = frame;
     obj.name = [obj.kinframe.name,'ForceTorque'];
   end

   function tf = isDirectFeedthrough(obj)
     tf = false;
   end

   function obj = compile(obj,tsmanip,manip)
     if (tsmanip.position_control) error('need to update this method for this case'); end

     try
       manip.findFrameId(obj.kinframe.name);
     catch ex
       if strcmp(ex.identifier,'Drake:RigidBodyManipulator:UniqueFrameNotFound')
         error('Drake:ContactForceTorqueSensor:FrameNotFound', ...
           ['The frame for this sensor (%s) could not be found on ',...
            'the manipulator'],obj.kinframe.name);
       else
         rethrow(ex)
       end
     end
     body = getBody(manip,obj.kinframe.body_ind);

     if isempty(body.contact_pts)
       error('Drake:ContactForceTorqueSensor:NoContactPts','There are no contact points associated with body %s',body.linkname);
     end

%      obj = setInputFrame(obj,MultiCoordinateFrame({getStateFrame(tsmanip),constructFrame(obj,}));

     nL = sum([manip.joint_limit_min~=-inf;manip.joint_limit_max~=inf]); % number of joint limits
     nC = manip.num_contacts;
     nP = 2*manip.num_position_constraints;  % number of position constraints
     nV = manip.num_velocity_constraints;  

     num_body_contacts = size(body.contact_pts,2);
     contact_ind_offset = size([manip.body(1:obj.kinframe.body_ind-1).contact_pts],2);

     % z(nL+nP+(1:nC)) = cN
     obj.normal_ind = nL+nP+contact_ind_offset+(1:num_body_contacts);

     mC = 2*length(manip.surfaceTangents(manip.gravity)); % get number of tangent vectors

     % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
     for i=1:mC
       obj.tangent_ind{i} = nL+nP+(mC*nC)+contact_ind_offset+(1:num_body_contacts);
     end

%     obj.manip = manip;
   end

%    function draw(obj,t,xft)
%      if length(obj.xyz)~=2
%        error('only implemented for the planar case so far');
%      end
% 
%      xft = splitCoordinates(getInputFrame(obj),xft);
%      x = xft{1}; ft = xft{2};
% 
%      kinsol = doKinematics(obj.manip,x(1:obj.manip.getNumDOF),false,false);
% 
%      body_pts = kinsol.T{obj.body}\[obj.xyz, obj.xyz+.001*ft(1:2); 1 1];  % convert force from sensor coords to body coords
%      body_pts = body_pts(1:2,:);
% 
%      world_pts = forwardKin(obj.manip,kinsol,obj.body,body_pts);
% 
%      figure(1); clf; xlim([-2 2]); ylim([-1 3]); axis equal;
%      axisAnnotation('arrow',world_pts(1,:),world_pts(2,:),'Color','r','LineWidth',2);
%    end

   function y = output(obj,tsmanip,frame_idx,t,x,u)
     cv = tsmanip.getStateFrame().splitCoordinates(x);
     y = cv{frame_idx};
   end

   function [tsmanip,xdn,df] = update(obj,tsmanip,t,x,u)
     if (nargout>2)
       error('ContactForceTorqueSensor:NoGradients',...
         'Gradients not yet supported for ContactForceTorqueSensor/update');
     end
     
     [tsmanip,z] = tsmanip.solveLCP(t,x,u);
     z = z/tsmanip.timestep;

     % todo: could do this more efficiently by only computing everything
     % below for indices where the normal forces are non-zero

     body = tsmanip.getBody(obj.kinframe.body_ind);

     % todo: Remove this copy of the TimeSteppingRigidBodyManipulator's
     % internal RigidBodyManipulator
     manip = tsmanip.getManipulator();
     
     kinsol = doKinematics(manip,x(1:manip.getNumDOF));
     contact_pos = forwardKin(manip,kinsol,obj.kinframe.body_ind,body.contact_pts);

     [d,N] = size(contact_pos);
     [pos,~,normal] = collisionDetect(manip,contact_pos);

     % flip to sensor coordinates
     pos = bodyKin(manip,kinsol,findFrameId(manip,obj.kinframe.name),pos);
     sensor_pos = forwardKin(manip,kinsol,findFrameId(manip,obj.kinframe.name),zeros(3,1));
     normal = bodyKin(manip,kinsol,findFrameId(manip,obj.kinframe.name),repmat(sensor_pos,1,N)+normal);
     tangent = manip.surfaceTangents(normal);

     % compute all individual contact forces in sensor coordinates
     force = repmat(z(obj.normal_ind)',d,1).*normal;
     mC=length(tangent);
     for i=1:mC
       force = force + repmat(z(obj.tangent_ind{i})',d,1).*tangent{i} ...
         - repmat(z(obj.tangent_ind{i+mC})',d,1).*tangent{i}; 
     end
     xdn = [ sum(force,2); sum(cross(pos,force),2) ];

     if tsmanip.twoD
       % y(1) = dot(force,x_axis)
       % y(2) = dot(force,y_axis)
       % y(3) = dot(torque,view_axis)
       % note: need to think / test whether this is correct torque for nontrivial
       % orientations of the sensor
       xdn = [[manip.x_axis'; manip.y_axis'],zeros(2,3); zeros(1,3),manip.view_axis']*xdn;
     end
   end
   
   function fr = constructFrame(obj,tsmanip)
     if tsmanip.twoD
       manip = getManipulator(tsmanip);
       coords{1}=['force_',manip.x_axis_label];
       coords{2}=['force_',manip.y_axis_label];
       coords{3}='torque';
       fr = CoordinateFrame(obj.name,3,'f',coords);
     else
       coords{1}='force_x';
       coords{2}='force_y';
       coords{3}='force_z';
       coords{4}='torque_x';
       coords{5}='torque_y';
       coords{6}='torque_z';
       fr = CoordinateFrame(obj.name,6,'f',coords);
     end
   end%
   
   function fr = constructStateFrame(obj,tsmanip)
     if tsmanip.twoD
       manip = getManipulator(tsmanip);
       coords{1}=['force_',manip.x_axis_label];
       coords{2}=['force_',manip.y_axis_label];
       coords{3}='torque';
       fr = CoordinateFrame([obj.name '_state'],3,'f',coords);
     else
       coords{1}='force_x';
       coords{2}='force_y';
       coords{3}='force_z';
       coords{4}='torque_x';
       coords{5}='torque_y';
       coords{6}='torque_z';
       fr = CoordinateFrame([obj.name '_state'],6,'f',coords);
     end
   end
   
   function x0 = getInitialState(obj,tsmanip)
     if tsmanip.twoD
       x0 = zeros(3,1);
     else
       x0 = zeros(6,1);
     end
   end
 end

end
