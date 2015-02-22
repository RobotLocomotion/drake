classdef FootContactMixin 
  properties
    contact_threshold = 0.001;
    using_flat_terrain = false;
    robot
    mex_ptr;
  end

  methods
    function obj = FootContactMixin(r, options)
      obj = applyDefaults(obj, options);
      obj.robot = r;

      terrain = getTerrain(r);
      if isa(terrain,'DRCTerrainMap') 
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end
      
      if exist('supportDetectmex','file')~=3
        error('can''t find supportDetectmex.  did you build it?');
      end      
      obj.mex_ptr = SharedDataHandle(supportDetectmex(0,r.getMexModelPtr.ptr,terrain_map_ptr));
    end

    function supp = getActiveSupports(obj, x, supp, contact_sensor)
      obj.robot.warning_manager.warnOnce('Drake:NotConsideringContactBreak', 'not considering breaking contact');
      contact_logic_AND = true;
      
      if ~obj.using_flat_terrain
        height = getTerrainHeight(obj.robot,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end

      obj.robot.warning_manager.warnOnce('Drake:UselessObject', 'creating a useless object just because we cannot pass in a struct');
      supp_state = RigidBodySupportState(supp);
      active_supports = supportDetectmex(obj.mex_ptr.data,x,supp_state,contact_sensor,obj.contact_threshold,height,contact_logic_AND);
      fc = [1.0*any(active_supports==obj.robot.foot_body_id.left); 1.0*any(active_supports==obj.robot.foot_body_id.right)];

      mask = true(1, length(supp.bodies));
      if ~fc(1)
        mask = mask & (~(supp.bodies == obj.robot.foot_body_id.left));
      end
      if ~fc(2)
        mask = mask & (~(supp.bodies == obj.robot.foot_body_id.right));
      end
      if ~all(mask)
        supp.bodies = supp.bodies(mask);
        supp.contact_pts = supp.contact_pts(mask);
        supp.contact_groups = supp.contact_groups(mask);
        supp.num_contact_pts = supp.num_contact_pts(mask);
        supp([supp.bodies] == obj.robot.foot_body_id.left) = [];
      end
    end
  end
end