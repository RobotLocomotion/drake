function sys = plantControlViz(plant,control,viz)

typecheck(plant,'DynamicalSystem');
typecheck(control,'DynamicalSystem');
typecheck(viz,'DynamicalSystem');

sys = Cascade(Feedback(plant,control),viz);
