clc
clear
close all

%% Check Ct
syms p0 v0 a0 j0 p1 p2 p3 v3 a3 j3 a1 v1 j1 a2 v2 j2 real
dfdp = [p0 v0 a0 j0 p1 p2 p3 v3 a3 j3, v1 a1 j1, v2 a2 j2]';
d = [p0 v0 a0 j0 p1 v1 a1 j1 p1 v1 a1 j1 p2 v2 a2 j2 p2 v2 a2 j2 p3 v3 a3 j3]';
Ct = getCt(3, 7);
dd = Ct*dfdp;
assert(isequaln(d,dd))
clear

%%
waypoints = [9 17 51 78 58]';
n_seg = length(waypoints) - 1;
n_order = 7;
ts = ones(n_seg, 1);

start_cond = [waypoints(1), 0, 0, 0];
end_cond   = [waypoints(end), 0, 0, 0];
Q = getQ(n_seg, n_order, ts);

M = getM(n_seg, n_order, ts);
Ct = getCt(n_seg, n_order);

%% check all
gold = load('hw2Data.mat');
assert(isequaln(Q,gold.Q));
assert(isequaln(Ct,gold.Ct))
%disp("Map matrix size: " + size(M));
%disp("gold size: " + size(gold.M));
disp("m1 : " + M(8,:));
disp("gm1: " + gold.M(8,:));
for i  = 1:size(M, 1)
    m1 = M(i,:);
    gm1 = gold.M(i,:);
    assert(isequal(m1, gm1));
    disp("i checked done. [" + i + "]");
end
assert(isequaln(M,gold.M))

disp('[TEST] hw2 pass')