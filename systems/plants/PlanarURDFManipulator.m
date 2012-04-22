classdef PlanarURDFManipulator < ManipulatorPlant
  % This class wraps the planar pieces of the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties (SetAccess=private,GetAccess=private)  
    model;     % system model data structure for Featherstone's spatial library
    urdf_filename;
  end
  
  methods
    function obj = PlanarURDFManipulator(urdf_filename)
      obj = obj@ManipulatorPlant(0,0,0);

      if (nargin<1)
        urdf_filename=uigetfile('*.urdf');
      end
      obj.model = parseURDF(urdf_filename);
      obj.urdf_filename=urdf_filename;
      
      obj = obj.setNumInputs(size(obj.model.B,2));
      num_loops = length(obj.model.loop);
      obj = obj.setNumDOF(obj.model.NB);
      obj = obj.setNumBilateralConstraints(2*num_loops);
    end
    
%   Note: will need to implement the constraint forces here to uncomment    
%     function xdot = dynamics(obj,t,x,u)
%       q = x(1:obj.num_q);
%       qd = x(obj.num_q+1:end);
%       tau = obj.model.damping'.*qd;
%       if (obj.num_u) 
%         tau = tau+obj.model.B*u;
%       end
%       xdot = [qd; FDabp(obj.model,q,qd,tau,{},obj.model.gravity)];
%     end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      [H,C] = HandCp(obj.model,q,qd,{},obj.model.gravity);
      C=C+obj.model.damping'.*qd;
      B = obj.model.B;
    end

    function phi = bilateralConstraints(obj,q)
      % note: each loop adds two constraints 
      phi=[];
      for i=1:length(obj.model.loop)
        % for each loop, add the constraints on T1(q) and T2(q), // todo: finish this
        % where
        % T1 is the transformation from the least common ancestor to the
        % constraint in link1 coordinates
        % T2 is the transformation from the least common ancester to the
        % constraint in link2 coordinates
        T1 = obj.model.loop(i).T1;
        link=obj.model.loop(i).link1;
        while (link~=obj.model.loop(i).least_common_ancestor)
          TJ = Tjcalcp(obj.model.jcode(link),q(link));
          T1=obj.model.Ttree{link}*TJ*T1;
          link = obj.model.parent(link);
        end
        T2 = obj.model.loop(i).T2;
        link=obj.model.loop(i).link2;
        while (link~=obj.model.loop(i).least_common_ancestor)
          TJ = Tjcalcp(obj.model.jcode(link),q(link));
          T2=obj.model.Ttree{link}*TJ*T2;
          link = obj.model.parent(link);
        end
        
        if (obj.model.loop(i).jcode==1)  % pin joint adds constraint that the transformations must match in position at the origin
          phi = [phi; [1,0,0; 0,1,0]*(T1*[0;0;1] - T2*[0;0;1])];
        else
          error('not implemented yet');
        end
      end
    end
    
    function v=constructVisualizer(obj)
      v = PlanarURDFVisualizer(obj.urdf_filename);
    end
  end
  
end

