classdef HybridSystemSampleTimeTest < DrakeSystem
  
  properties 
    sample_time
  end
  
  methods 
    function obj=HybridSystemSampleTimeTest(ts)
      obj = obj@DrakeSystem(0,0,0,0,false,true);
      obj.sample_time = ts;
    end
    
    function ts = getSampleTime(obj)
      ts = obj.sample_time;
    end
  end
  
  methods (Static=true)
    function run()
      % run a barrage of tests on sample time merging behavior
      
      sys=HybridDrakeSystem(0,0);
      sys=addMode(sys,HybridSystemSampleTimeTest([-1;0]));
      sys=addMode(sys,HybridSystemSampleTimeTest([-1;0]));
      ts = sys.getSampleTime();
      if (~isequal(ts,[-1;0])) error('merging inherited sample times should result in inherited sample time'); end

      sys=HybridDrakeSystem(0,0);
      sys=addMode(sys,HybridSystemSampleTimeTest([0;0]));
      sys=addMode(sys,HybridSystemSampleTimeTest([-1;0]));
      ts = sys.getSampleTime();
      if (~isequal(ts,[0;0])) error('hybrid sample times error'); end

%     % disabled support for mixed sample times... at least for now.
%      sys=HybridDrakeSystem(0,0);
%      sys=addMode(sys,HybridSystemSampleTimeTest([0;0]));
%      sys=addMode(sys,HybridSystemSampleTimeTest([-1;0]));
%      sys=addMode(sys,HybridSystemSampleTimeTest([2;2]));
%      ts = sys.getSampleTime();
%      if (~isequal(ts,[0 2;0 2])) error('hybrid sample times error'); end

      sys=HybridDrakeSystem(0,0);
      sys=addMode(sys,HybridSystemSampleTimeTest([-1;0]));
      sys=addMode(sys,HybridSystemSampleTimeTest([2;2]));
      ts = sys.getSampleTime();
      if (~isequal(ts,[2;2])) error('hybrid sample times error'); end

      sys=HybridDrakeSystem(0,0);
      sys=addMode(sys,HybridSystemSampleTimeTest([0;0]));
      failed=false;
      try 
        sys=addMode(sys,HybridSystemSampleTimeTest([0;1]));
      catch
        failed=true;
      end
      if (~failed) error('hybrid sample times error'); end

      sys=HybridDrakeSystem(0,0);
      sys=addMode(sys,HybridSystemSampleTimeTest([1;0]));
      failed=false;
      try 
        sys=addMode(sys,HybridSystemSampleTimeTest([1;1]));
      catch
        failed=true;
      end
      if (~failed) error('hybrid sample times error'); end
      
      sys=HybridDrakeSystem(0,0);
      sys=addMode(sys,HybridSystemSampleTimeTest([2;0]));
      sys=addMode(sys,HybridSystemSampleTimeTest([-1;0]));
      failed=false;
      try 
        sys=addMode(sys,HybridSystemSampleTimeTest([1;0]));
      catch
        failed=true;
      end
      if (~failed) error('hybrid sample times error'); end
      
    end
  end
end
