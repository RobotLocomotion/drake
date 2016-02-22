classdef CollisionFilterGroup
  properties
    warning_manager
    members={}
    ignored_collision_fgs={};
    %collides_with = CollisionFilterGroup.ALL_COLLISION_FILTER_GROUPS;
  end
  properties (Constant)
    % Collision filter values are 16-bit bitmasks
    DEFAULT_COLLISION_FILTER_GROUP_ID = true;
    NO_COLLISION_FILTER_GROUPS = logical.empty();
  end
  methods
    function obj = CollisionFilterGroup()
      obj.warning_manager = WarningManager();
    end

    function obj = addMembers(obj,new_members,robotnum)
      if isnumeric(robotnum)
        robotnum_str = num2str(robotnum);
      elseif iscell(robotnum)
        robotnum_str = reshape(cellfun(@num2str,robotnum,'UniformOutput',false),1,[]);
      else
        error('CollisionFilterGroup:addMembers:robotnum',...
              'robotnum must be a numeric type or a cell array');
      end
      new_members = reshape(new_members,1,[]);
      new_members = strcat(new_members,strcat('__',robotnum_str));
      obj.members = union(obj.members,new_members);
    end

    function obj = removeMembers(obj,members,robotnum)
      if isnumeric(robotnum)
        robotnum_str = num2str(robotnum);
      elseif iscell(robotnum)
        robotnum_str = reshape(cellfun(@num2str,robotnum,'UniformOutput',false),1,[]);
      else
        error('CollisionFilterGroup:addMembers:robotnum',...
              'robotnum must be a numeric type or a cell array');
      end
      members = reshape(members,1,[]);
      members = strcat(members,strcat('__',robotnum_str));
      obj.members = setdiff(obj.members,members);
    end

    function [linknames,robotnums] = getMembers(obj)
      if ~isempty(obj.members)
        % Split at last occurence of separator
        result = regexp(obj.members,'__(?!.*__)','split');
        result = vertcat(result{:})';
        linknames = result(1,:);
        robotnums = cellfun(@str2num,result(2,:),'UniformOutput',false);
      else
        linknames = {};
        robotnums = {};
      end
    end

    function ignored_collision_fgs = getIgnoredCollisionFilterGroups(obj)
      ignored_collision_fgs = reshape(obj.ignored_collision_fgs,1,[]);
    end

    function obj = updateForRemovedLink(obj,model,child_index,new_parent_linkname,collision_fg_name)
      % Get parent info
      parent_index = model.getBody(child_index).parent;
      old_parent_linkname = model.getLinkName(parent_index);
      parent_robotnum = model.getBody(parent_index).robotnum;
      add_parent = false;

      % Get group members
      [linknames,robotnums] = obj.getMembers();

      % Check if the parent is in the group
      if any(strcmp(linknames,old_parent_linkname) & cellfun(@(num) num == parent_robotnum,robotnums))
        % Remove the old parent link
        obj = obj.removeMembers(old_parent_linkname,parent_robotnum);

        % Update parent link name
        add_parent = true;
      end

      % Check if the child is in the group
      linkname = model.getLinkName(child_index);
      robotnum = model.getBody(child_index).robotnum;

      if any(strcmp(linknames,linkname) & cellfun(@(num) num == robotnum,robotnums))
        % Remove child link
        obj = obj.removeMembers(linkname,robotnum);

        % Add the parent link if the parent has no collision geometry but the child
        % does
        if ~add_parent && ~isempty(model.getBody(child_index).collision_geometry)
          if isempty(model.getBody(parent_index).collision_geometry)
            add_parent = true;
          else
            warnOnce(obj.warning_manager, ...
              'Drake:CollisionFilterGroup:DiscardingCollisionFilteringInfo', ...
              ['The body ''%s'' belongs to the collision filter group ''%s'', but ' ...
              'has been welded to its parent, ''%s'', which does not. ' ...
              'The welded body, ''%s'', will not belong to ''%s.'''], ...
              linkname, collision_fg_name, old_parent_linkname, ...
              new_parent_linkname, collision_fg_name);
          end
        end
      end

      % Add the parent to the group if necessary
      if add_parent
        obj = obj.addMembers(new_parent_linkname,parent_robotnum);
      end
    end
  end

  methods (Static)
    function testAddGetMembers()
      collision_fg = CollisionFilterGroup();
      linknames1 = {'abcd';'efgh'};
      robotnums1 = {1,2};
      linknames2 = {'hijk'};
      robotnums2 = 1;
      linknames3 = 'lmno';
      robotnums3 = 1;
      linknames4 = {'pqrs';'tuvw'};
      robotnums4 = 2;
      linknames_ref = {'abcd','efgh','hijk','lmno','pqrs','tuvw'};
      robotnums_ref = {1,2,1,1,2,2};
      collision_fg = addMembers(collision_fg,linknames1,robotnums1);
      collision_fg = addMembers(collision_fg,linknames2,robotnums2);
      collision_fg = addMembers(collision_fg,linknames3,robotnums3);
      collision_fg = addMembers(collision_fg,linknames4,robotnums4);
      [linknames,robotnums] = getMembers(collision_fg);
      assert(all(strcmp(linknames,linknames_ref)));
      assert(all(isequal(robotnums,robotnums_ref)));
    end

    function testAddRepeatedly()
      collision_fg = CollisionFilterGroup();
      linknames1 = {'abcd','efgh'};
      linknames2 = {'abcd','efgh'};
      robotnums1 = {1,2};
      robotnums2 = {2,2};
      linknames_ref = {'abcd','abcd','efgh'};
      robotnums_ref = {1,2,2};
      collision_fg = addMembers(collision_fg,linknames1,robotnums1);
      collision_fg = addMembers(collision_fg,linknames2,robotnums2);
      [linknames,robotnums] = getMembers(collision_fg);
      assert(all(strcmp(linknames,linknames_ref)));
      assert(all(isequal(robotnums,robotnums_ref)));
    end

    function testRemoveMember()
      collision_fg = CollisionFilterGroup();
      linknames1 = {'abcd','efgh'};
      robotnums1 = {1,2};
      linknames2 = {'efgh'};
      robotnums2 = 2;
      linknames_ref = {'abcd'};
      robotnums_ref = {1};
      collision_fg = addMembers(collision_fg,linknames1,robotnums1);
      collision_fg = removeMembers(collision_fg,linknames2,robotnums2);
      [linknames,robotnums] = getMembers(collision_fg);
      assert(all(strcmp(linknames,linknames_ref)));
      assert(all(isequal(robotnums,robotnums_ref)));
    end
  end
end
