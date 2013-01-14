classdef ContactForceTorqueSensor < RigidBodySensor
  
  properties
    frame;
    tsmanip
    body
  end
  
  methods
    function obj = ContactForceTorqueSensor(tsmanip,body,xyz,rpy)
      typecheck(tsmanip,'TimeSteppingRigidBodyManipulator');
      typecheck(body,'RigidBody');
      
      if isempty(body.contact_pts)
        error('Drake:ContactForceTorqueSensor:NoContactPts','There are no contact points associated with body %s',body.name);
      end

      if isa(body,'PlanarRigidBody')
        typecheck(tsmanip.manip,'PlanarRigidBodyManipulator'); 
        coords{1}=['force_',tsmanip.manip.x_axis_label];
        coords{2}=['force_',tsmanip.manip.y_axis_label];
        coords{3}='torque';
      else
        coords{1}='force_x';
        coords{2}='force_y';
        coords{3}='force_z';
        coords{4}='torque_x';
        coords{5}='torque_y';
        coords{6}='torque_z';
      end
      obj.frame = CoordinateFrame([body.linkname,'ForceTorqueSensor'],length(coords),'f',coords);
      obj.tsmanip = tsmanip;
      obj.body = body;
    end
    function y = output(obj,t,x,u)
      error('not implemented yet');
    end
    function fr = getFrame(obj)
      fr = obj.frame;
    end
  end
  
end