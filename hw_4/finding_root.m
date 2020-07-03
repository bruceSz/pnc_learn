
function res = finding_root(p, n)
    A =  diag(ones(n-1, 1),-1)
    A(1,:) = -p(2:n+1)./p(1)
    res = eig(A);
end


