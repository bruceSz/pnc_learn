

% roots on p
p = [ 10 0 0 3 2];
disp("size of p is :"+ size(p,2));
roots(p);

% diag method
res = finding_root(p, size(p,2)-1)

for i = 1:size(res,1)
    disp("res is: " + res(i,:) )
end

%disp("finding_root is: " + size(res,1))