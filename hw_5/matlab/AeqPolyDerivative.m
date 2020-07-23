
% test this.
function res = AeqPolyDerivative(vals, n_order, base)
    res = [];
    coef_n = n_order + 1;

    for i =1: coef_n
        multi = i - 1;
        res(i)  = multi * vals(i);
        res(i) = res(i) / base;
    end
end
