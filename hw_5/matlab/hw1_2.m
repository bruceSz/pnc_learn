clc;clear;close all;
path = ginput() * 100.0;

n_order = 7;
n_seg = size(path, 1) - 1;
n_poly_perseg = n_order + 1;

ts = zeros(n_seg, 1);
% calculate time distribution based on distance between 2 points
dist = zeros(n_seg, 1);
dist_sum = 0;
T = 25;

t_sum = 0;
for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum + dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i) / dist_sum * T;
    t_sum = t_sum + ts(i);
end
ts(n_seg) = T - t_sum;
% or you can simply average the time
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

poly_coef_x = MinimumSnapCloseformSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapCloseformSolver(path(:, 2), ts, n_seg, n_order);

poly_coef_x_p = poly_coef_x';
poly_coef_y_p = poly_coef_y';

poly_coef_x_p = poly_coef_x_p(1,:);
poly_coef_y_p = poly_coef_y_p(1,:);
disp("size of poly coef_x: " + size(poly_coef_x_p));
disp("size of poly coef y: " + size(poly_coef_y_p));

%disp("1  x: " + poly_coef_x_p(1,:));
%disp("2  x: " + poly_coef_x_p(2,:));
%disp("3  x: " + poly_coef_x_p(3,:));


X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 4: get the coefficients of i-th segment of both x-axis
    % and y-axis

    skip = i*(n_order + 1);
    %disp("skip is : " + skip);

    
    tmp_x = poly_coef_x_p(1,skip+1: skip+n_order+1);
    Pxi = flip(tmp_x);
    sqx = sumsqr(Pxi);
    %disp("sqx is: " + sqx);

    tmp_y = poly_coef_y_p(1,skip+1: skip+n_order+1);
    Pyi  =  flip(tmp_y);
    sqy = sumsqr(Pyi);
    %disp("sqy is: " + sqy);


    %Pxi = [];
    %Pyi = [];
    for t=0:tstep:ts(i+1)
        X_n(k)  = polyval(Pxi,t);
        Y_n(k)  = polyval(Pyi,t);
        k = k+1;
    end
end



%disp("X_n is : " + X_n);
plot(X_n, Y_n ,'Color',[0 1.0 0],'LineWidth',2);
hold on
scatter(path(1:size(path,1),1),path(1:size(path,1),2));

function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond =   [waypoints(end), 0, 0, 0];
    %#####################################################
    % you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 1: compute M
    M = getM(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Ct
    Ct = getCt(n_seg, n_order);
    C = Ct';
    R = C * inv(M)' * Q * inv(M) * Ct;
    disp("size of R row : " + size(R,1) + " size of R col: " + size(R,2));
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    %#####################################################
    % STEP 3: compute dF
    % dF = [];
    dF = zeros(n_seg + 7);
    %
    %
    %
    %
    dF(1,1) = start_cond(1,1);
    dF(2,1) = start_cond(1,2);
    dF(3,1) = start_cond(1,3);
    dF(4,1) = start_cond(1,4);

    dF(n_seg + 4,1) = end_cond(1,1);
    dF(n_seg + 5,1) = end_cond(1,2);
    dF(n_seg + 6,1) = end_cond(1,3);
    dF(n_seg + 7,1) = end_cond(1,4);

    % size of wp equals to n_seg + 1.
    for i=1:n_seg -1;
        wp_idx = i+1;
        dF(4+i,1) = waypoints(wp_idx);
    end
    
    disp("rpp is: " + size(R_pp,2));
    disp( " rfp  prime is: " + size(R_fp',1) );

    dP = -inv(R_pp) * (R_fp') * dF;
    poly_coef = inv(M) * Ct * [dF;dP];
    %disp("p coef is: " + poly_coef);
end