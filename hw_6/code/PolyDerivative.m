
% test this.
function res = PolyDerivative(coefs, n_order, base)
    % here the coefs should be from high order to low order.
   res = [];
   coefs_len = size(coefs, 2);
   coef_n = n_order + 1;

  for i = n_order:-1:1
       idx = n_order  - i;
       res(idx+1) = coefs(idx + 1) * i / base;

  end
  
  diff = coefs_len - n_order;
  %disp("diff is : " + diff + "coef_n is: " + coef_n + " diff is: " + n_order);
  for i = 1 : diff
       idx = i + n_order;
       res(idx) = 0;
  end
  %
end
