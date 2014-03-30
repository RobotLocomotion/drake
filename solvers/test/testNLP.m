function testNLP
nlp1 = NLPtest1();
nlp1 = nlp1.setCheckGrad(true);
[x,F,info] = nlp1.solve([1;2]);
valuecheck(x,[0;-1],1e-5);
[x,F,info] = nlp1.compareSolvers([1;2]);
nlp2 = NLPtest2();
nlp2 = nlp2.setCheckGrad(true);
[x,F,info] = nlp2.solve([1;2]);
valuecheck(x,[sqrt(2);sqrt(2)/2],1e-3);
[x,F,info] = nlp2.compareSolvers([1;2]);

nlp3 = NLPtest3();
nlp3 = nlp3.setCheckGrad(true);
[x,F,info] = nlp3.solve([1;2;3]);
[x,F,info] = nlp3.compareSolvers([1;2;3]);
end