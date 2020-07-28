a = [1 1 1 1];
res = PolyDerivative(a, 4, 1);
disp("after derivative is:" + res);
res2 = PolyDerivative(res, 3, 1);
disp("after derivative twice : " + res2);


res3 = PolyDerivative(res2, 2, 1);
disp("after derivative third t : " + res3);