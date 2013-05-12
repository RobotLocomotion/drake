classdef ContactAffordance
  properties
    name; % we use 'name' as the unique identifier for each contact affordance
    robot;
    body_ind;
    is_fixed = true;
    mu; % friction coefficient
  end
  methods
    function obj = ContactAffordance(robot,body_ind,name,is_fixed,mu)
      if nargin == 0
        return;
      end

      obj.robot = robot;

      sizecheck(body_ind,1);
      if(isa(body_ind,'RigidBody'))
        body_ind = robot.findLinkInd(body_ind.linkname);
      end
      obj.body_ind = body_ind;

      if (nargin>2)
        typecheck(name,'char');
        obj.name = name;
      end

      if (nargin>3)
        typecheck(is_fixed,'logical');
        obj.is_fixed = is_fixed;
%         assert(is_fixed, ...
%           'Only fixed contact affordances are supported at this time');
      end
      
      if (nargin>4)
          if(mu<0)
              error('Static friction coefficient should be nonnegative')
          end
          obj.mu = mu;
      else
          obj.mu = 1;
      end
    end
    function [res, dRes] = residualsPts2Aff(obj,q,pts)
        % res = [dist;add_res]. dist is the signed normal distance to the
        % affordance. add_res is used when the affordance has finite area.
        % For example, for polygon affordance, add_res is all positive if
        % the projection of the pts are all within the polygon
      [dist, dDist] = distancePts2Aff(obj,q,pts);
      [add_res, dAdd_res] = additionalResPts2Aff(obj,q,pts);
      res = [dist; add_res];
      dRes = [dDist; dAdd_res];
    end
    function [dist, dDist] = distancePts2Aff(obj,q,pts)
      dist = -1;
      dDist = 0;
    end
    function [res, dRes] = additionalResPts2Aff(obj,q,pts)
      res = [];
      dRes = [];
    end
    
    function mu = getFrictionCoef(obj)
        mu = obj.mu;
    end
  end
end
