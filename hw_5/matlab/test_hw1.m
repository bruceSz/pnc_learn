clc
clear
close all

%% check getQ
T = 1.0;

% In matlab scalar and single val list are the same.
% which is weird and tend to be confusing.

syms x p0 p1 p2 p3 p4 p5 p6 p7 real;

p = [p7 p6 p5 p4 p3 p2 p1 p0]; % high to low
f = poly2sym(p);

f2 = diff(f,4)*diff(f,4);
%disp("f2 is : " + f2);
J = int(f2);
Jt = subs(J, x, T)


Q = getQ(1,7,T);

%disp("q is " + Q); 
P = fliplr(p)'; % low to high

Jt2 = simplify(P'*Q*P)

assert(isequaln(Jt,Jt2))
clear

%%fli
waypoints = [9 17 51 78 58]';
n_seg = length(waypoints) - 1;
n_order = 7;
ts = ones(n_seg, 1);

start_cond = [waypoints(1), 0, 0, 0];
end_cond   = [waypoints(end), 0, 0, 0];
Q = getQ(n_seg, n_order, ts);
[Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);

%% check all
gold = load('hw1Data.mat');
assert(isequaln(Q,gold.Q));
disp("The Q is equal. Check");

%disp("gold.Aeq is : " + size(gold.Aeq));
%disp("my aeq is : " + size(Aeq));


assert(isequal(size(Aeq,1), size(gold.Aeq,1)));
assert(isequal(size(Aeq,2), size(gold.Aeq,2)));




disp("go Aeq: " +gold.Aeq(17,:));
disp("my Aeq: " +Aeq(17,:));
%disp("my Aeq: " + Aeq(6,:));

%disp("go Aeq: " +gold.Aeq(7,:));
%disp("my Aeq: " + Aeq(7,:));

assert(isequal(gold.Aeq(1,:), Aeq(1,:)));
assert(isequal(gold.Aeq(2,:), Aeq(2,:)));
assert(isequal(gold.Aeq(3,:), Aeq(3,:)));
assert(isequal(gold.Aeq(4,:), Aeq(4,:)));

assert(isequal(gold.Aeq(5,:), Aeq(5,:)));
assert(isequal(gold.Aeq(6,:), Aeq(6,:)));
assert(isequal(gold.Aeq(7,:), Aeq(7,:)));
assert(isequal(gold.Aeq(8,:), Aeq(8,:)));


for i = 1:size(gold.Aeq,1)
    assert(isequal(gold.Aeq(i,:), Aeq(i,:)));
    disp("i check equal checked done. " + i);
end


assert(isequaln(Aeq,gold.Aeq)|isequaln(Aeq,gold.Aeq2))

disp("Aeq is equal . check done.");
assert(isequaln(beq,gold.beq))
disp('[TEST] hw1 pass')