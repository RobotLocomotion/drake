classdef QuasiStaticConstraint<Constraint
% constrain the Center of Mass to lie inside the shrunk support polygon
% @param robot             -- The robot
% @param robotnum          -- The robotnum to compute CoM. Set to 1 if only intend to
%                             compute the CoM of robot(1)
% @param tspan             -- The time span of the constraint. An optional
%                             argument. If it is not passed in the constructor,
%                             then tspan defaults to [-inf inf];
% @param active            -- A flag, true if the quasiStaticFlag would be active
% @param num_bodies        -- An int, the total number of bodies that have active
%                             ground contact points
% @param bodies            -- An int array of size 1xnum_bodies. The index of each
%                             ground contact body
% @param num_body_pts      -- An int array of size 1xnum_bodies. The number of
%                             active contact points in each body
% @param body_pts          -- A cell array of size 1xnum_bodies. body_pts{i} is a
%                             3xnum_body_pts(i) double array, which is the active
%                             ground contact points in the body frame
  properties(SetAccess = protected)
    robot;
    robotnum;
    tspan;
    nq;
    shrinkFactor
    active;
    num_bodies;
    num_pts 
    bodies;
    num_body_pts; 
    body_pts;
    plane_row_idx;
    mex_ptr;
  end
  
  methods
    function obj = QuasiStaticConstraint(robot,robotnum,tspan)
      if(nargin <3)
        tspan = [-inf,inf];
      end
      tspan = [tspan(1) tspan(end)];
      ptr = constructPtrQuasiStaticConstraintmex(robot.getMexModelPtr,robotnum,tspan);
      obj = obj@Constraint(Constraint.QuasiStaticConstraintType);
      obj.robot = robot;
      if(~isempty(setdiff(robotnum,1:length(obj.robot.name))))
        error('Drake:QuasiStaticConstraint: robotnum is not accepted');
      end
      obj.robotnum = robotnum;
      if(tspan(1)>tspan(end))
        error('Drake:QuasiStaticConstraint:tspan(1) must be no larger than tspan(end)');
      end
      obj.tspan = tspan;
      obj.nq = robot.getNumDOF;
      obj.shrinkFactor = 0.9;
      obj.active = false;
      obj.num_bodies = 0;
      obj.num_pts = 0;
      obj.bodies = [];
      obj.num_body_pts = [];
      obj.body_pts = {};
      obj.plane_row_idx;
      obj.mex_ptr = ptr;
    end
    
    function flag = isTimeValid(obj,t)
      if(isempty(t))
        flag = true;
      else
        if(t>=obj.tspan(1)&&t<=obj.tspan(end))
          flag = true;
        else
          flag = false;
        end
      end
    end

    function obj = setActive(obj,flag)
      obj.active = logical(flag);
      updatePtrQuasiStaticConstraintmex(obj.mex_ptr,'active',obj.active)
    end
    
    function num_cnst = getNumConstraint(obj,t)
      if(obj.isTimeValid(t))
        num_cnst = 3;
      else
        num_cnst = 0;
      end
    end
    
    function obj = addContact(obj,varargin)
      % obj.addContact(body1,body1_pts,body2,body2_pts,...,bodyN,bodyN_pts)
      updatePtrQuasiStaticConstraintmex(obj.mex_ptr,'contact',varargin{:});
      i = 1;
      while(i<length(varargin))
        body = varargin{i};
        body_pts = varargin{i+1};
        if(isnumeric(body))
          sizecheck(body,[1,1]);
        elseif(ischar(body))
          body = obj.robot.findLinkInd(body);
        elseif(typecheck(body,'RigidBody'))
          body = obj.robot.findLinkInd(body.linkname);
        else
          error('Drake:QuasiStaticConstraint:Body must be either the link name or the link index');
        end
        body_idx = find(obj.bodies == body);
        if(isempty(body_idx))
          obj.bodies = [obj.bodies body];
          obj.num_bodies = obj.num_bodies+1;
          npts = size(body_pts,2);
          sizecheck(body_pts,[3,npts]);
          obj.body_pts = [obj.body_pts {body_pts}];
          obj.num_body_pts = [obj.num_body_pts npts];
          row_idx = bsxfun(@plus,[1;2;3],3*(0:npts-1));
          obj.plane_row_idx = [obj.plane_row_idx;obj.num_pts*3+reshape(row_idx(1:2,:),[],1)];
          obj.num_pts = obj.num_pts+npts;
        else
          num_body_pts_tmp = size(obj.body_pts{body_idx},2);
          obj.body_pts{body_idx} = (unique([obj.body_pts{body_idx} body_pts]','rows'))';
          obj.num_body_pts(body_idx) = size(obj.body_pts{body_idx},2);
          obj.num_pts = obj.num_pts-num_body_pts_tmp+obj.num_body_pts(body_idx);
          row_idx = reshape((1:3*obj.num_pts),3,obj.num_pts);
          obj.plane_row_idx = reshape(row_idx(1:2,:),[],1);
        end
        i = i+2;
      end
    end
    
    function obj = setShrinkFactor(obj,factor)
      updatePtrQuasiStaticConstraintmex(obj.mex_ptr,'factor',factor);
      typecheck(factor,'double');
      sizecheck(factor,[1,1]);
      if(factor<0)
        error('QuasiStaticConstraint: shrinkFactor should be non negative');
      end
      obj.shrinkFactor = factor;
    end
    
    function [c,dc] = eval(obj,t,kinsol,weights)
      if(obj.isTimeValid(t))
        [com,dcom] = obj.robot.getCOM(kinsol,obj.robotnum);
        contact_pos = zeros(3,obj.num_pts);
        dcontact_pos = zeros(3*obj.num_pts,obj.nq);
        num_accum_pts = 0;
        for i = 1:obj.num_bodies
          [contact_pos(:,num_accum_pts+(1:obj.num_body_pts(i))),...
            dcontact_pos(3*num_accum_pts+(1:3*obj.num_body_pts(i)),:)] = forwardKin(obj.robot,kinsol,obj.bodies(i),obj.body_pts{i},0);
          num_accum_pts = num_accum_pts+obj.num_body_pts(i);
        end
        plane_contact_pos = contact_pos(1:2,:);
        dplane_contact_pos = dcontact_pos(obj.plane_row_idx,:);
        center_pos = mean(plane_contact_pos,2);
        dcenter_pos = [mean(dplane_contact_pos(1:2:end,:),1);mean(dplane_contact_pos(2:2:end,:),1)];
        support_pos = plane_contact_pos*obj.shrinkFactor+bsxfun(@times,center_pos*(1-obj.shrinkFactor),ones(1,obj.num_pts));
        dsupport_pos = dplane_contact_pos*obj.shrinkFactor+repmat(dcenter_pos*(1-obj.shrinkFactor),obj.num_pts,1);
        c = com(1:2,:)-support_pos*weights;
        dc = [dcom(1:2,:)-[weights'*dsupport_pos(1:2:end,:);weights'*dsupport_pos(2:2:end,:)] -support_pos];
      else
        c = [];
        dc = [];
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = [0;0];
        ub = [0;0];
      else
        lb = [];
        ub = [];
      end
    end
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        if(isempty(t))
          name_str = {sprintf('QuasiStaticConstraint x');sprintf('QuasiStaticConstraint y')};
        else
          name_str = {sprintf('QuasiStaticConstraint x at time %10.4f',t);sprintf('QuasiStaticConstraint y at time %10.4f',t)};
        end
      else
        name_str = [];
      end
    end
    
    function obj = updateRobotnum(obj,robotnum)
      if(~isempty(setdiff(robotnum,1:length(obj.robot.name))))
        error('Drake:QuasiStaticConstraint: robotnum is not accepted');
      end
      obj.robotnum = robotnum;
      updatePtrQuasiStaticConstraintmex(obj.mex_ptr,'robotnum',robotnum);
    end
    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj.nq = obj.robot.getNumDOF();
      updatePtrQuasiStaticConstraintmex(obj.mex_ptr,'robot',obj.robot.getMexModelPtr);
    end
    
  end
end
