% test this.
function res = getAeqPoly(n_order, tval) 
    res = [];
    for i = 1:n_order
        res(i) = tval^(i);
    end
end