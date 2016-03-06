function simpleMixedCTDTtest

oldpath = addpath(fullfile(pwd,'..'));

try 
  sys = SimpleMixedCTDTExample();
  dtsys = SimpleDTExample();
  ctsys = SimpleCTExample();

  for i=1:10  % run the test 10 times with random initial conditions
    x0 = 2*rand(2,1)-1;  % unstable for |x|>1
    xtraj = simulate(sys,[0 3],x0);
    xtrajdt = simulate(dtsys,[0 3],x0(1));
    xtrajct = simulate(ctsys,[0 3],x0(2));

    xf=eval(xtraj,2);
    if (any(abs(xf-[eval(xtrajdt,2);eval(xtrajct,2)])>1e-6))
    
      figure(1); clf
      subplot(1,2,1);
      fnplt(xtraj,1);  % plot DT x
      hold on;
      h=fnplt(xtrajdt); set(h,'Color','r');
      xlim([0 5]);
      
      subplot(1,2,2);
      fnplt(xtraj,2);  % plot CT x
      hold on;
      h=fnplt(xtrajct); set(h,'Color','r','LineStyle','.');
      xlim([0 5]);
      
      error('this mixed trajectory solution should be the same as if the CT and DT systems were simulated by themselves');
    end
  end
  
catch err
  path(oldpath);
  rethrow(err)
end

path(oldpath);
