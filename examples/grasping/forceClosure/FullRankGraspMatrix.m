classdef FullRankGraspMatrix < BMIspotless
  properties(SetAccess = protected)
    xc % A 3 x nc matrix
    XC % A nc x 1 cell, XC{i} is supposed to be the bilinear matrix xc(:,i)*xc(:,i)'
  end
  
  methods
    function obj = FullRankGraspMatrix(nc)
      obj = obj@BMIspotless();
      [obj,obj.xc] = obj.newFree(3,nc);
      G = graspTransferMatrix(obj.xc);
      obj.XC = cell(nc,1);
      GG = zeros(6,6);
      for i = 1:nc
        [obj,obj.XC{i}] = obj.newSym(3);
        GG = GG+reshape(replaceBilinearProduct(G(:,(i-1)*3+(1:3))*G(:,(i-1)*3+(1:3))',obj.xc(:,i),obj.XC{i}),6,6);
        obj = obj.addBilinearVariable(obj.xc(:,i),obj.XC{i});
      end
      alpha = 1e-2;
      obj = obj.withPSD(GG-alpha*eye(6));
    end
  end
end