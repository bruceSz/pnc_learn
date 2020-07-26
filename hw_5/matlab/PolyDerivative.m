
% test this.
function res = PolyDerivative(coefs, n_order, base)
    res = [];
    coef_n = n_order + 1;

   for i = n_order:-1:1
        idx = n_order  - i;
        res(idx+1) = coefs(idx + 1) * i / base;

   end

   %
end
