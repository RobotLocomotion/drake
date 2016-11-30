function testSimInput
  
sys = LinearSystem([],[],[],[],[],eye(2));

sys2 = SimulinkModel(sys.getModel());

for i=1:10
  u = randn(2,1);
  valuecheck(output(sys2,rand,[],u),u);
end
