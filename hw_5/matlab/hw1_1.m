clc;clear;close all;
path = ginput() * 100.0;

n_order       = 7;% order of poly
n_seg         = size(path,1)-1;% segment number
disp("size of path is " + n_seg );
n_poly_perseg = (n_order+1); % coef number of perseg

ts = zeros(n_seg, 1);

for i = 1:size(path,1)
    disp(i + " point is: " + path(i,:))
end

% calculate time distribution in proportion to distance between 2 points
% dist     = zeros(n_seg, 1);
% dist_sum = 0;
% T        = 25;
% t_sum    = 0;
% 
% for i = 1:n_seg
%     dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
%     dist_sum = dist_sum+dist(i);
% end
% for i = 1:n_seg-1
%     ts(i) = dist(i)/dist_sum*T;
%     t_sum = t_sum+ts(i);
% end
% ts(n_seg) = T - t_sum;

% or you can simply set all time distribution as 1
for i = 1:n_seg
    ts(i) = 1.0;
end

poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);

poly_coef_x_p = poly_coef_x';
poly_coef_y_p = poly_coef_y';

disp("coef_x_p size: " + size(poly_coef_x_p));
disp("coef_y_p size: " + size(poly_coef_y_p));

% display the trajectory
X_n_n = [];
Y_n_n = [];
k = 1;
tstep = 0.01;

disp("all together has " + n_seg + " segments.");
coef_n  = n_order + 1;

for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
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

    
    j = 1;
    X_n = [];
    Y_n = [];

    
    for t = 0:tstep:ts(i+1)
        
        X_n(j)  = polyval(Pxi, t);
        Y_n(j)  = polyval(Pyi, t);
        if t == 0
            disp("vx of seg : " + i + " is : " + tmp_x(2));
            disp("vy of seg : " + i + " is : " + tmp_y(2));
            %disp(" t on segment 0:" + t);
            %disp("x is : " + polyval(Pxi, t));
            %disp("y is : " + polyval(Pyi, t));
            %disp(" cx is : " + Pxi);
            %disp(" cy is : " + Pyi);

        end

        if t == 1
            xxx = 1;
        end

        k = k + 1;
        j = j + 1;
    end

    X_n_n(i+1,:) = X_n;
    Y_n_n(i+1,: ) = Y_n;

end

vals = getAeqPoly(n_order, 1);


for i=0:n_seg-1
    skip = i*(n_order + 1);
    disp("skip is : " + skip);

    tmp_x = poly_coef_x_p(1,skip+1: skip+n_order+1);
    poly_cx =flip(tmp_x);
    tmp_y = poly_coef_y_p(1,skip+1: skip+n_order+1);
    poly_cy = flip(tmp_y);





    %%disp("pxb of seg is : " + polyval(poly_cx,0));
    %disp("pyb of seg is : " + polyval(poly_cy,0));
    disp("vx1_b is : " + tmp_x(2));
    disp("vy1_b is : " + tmp_y(2));


    %%%%
    if i > 0
        con_v = zeros(1, coef_n * 2);
        tmp = [1 vals];
        v_vals = AeqPolyDerivative(tmp,n_order, 1);

        con_v(1,1 : coef_n ) = v_vals;
        con_v(1, coef_n+2) = -1;

        two_seg_coef_x = [ p_coef_x tmp_x];
        res = con_v * two_seg_coef_x';
        disp("here the res should be 0, and it is: " + res);
        p_p = tmp * p_coef_x';
        p_v = v_vals * p_coef_x';
        disp("before p is : " + p_p);
        disp("before_vx is : " + p_v);
        tmp_coef = zeros(1, coef_n);
        tmp_coef(1,2) = 1;
        c_v = tmp_coef * tmp_x';
        disp("after p is : " + tmp_x(1));
        disp("after_vx is : " + c_v);
        %disp("after vxxxx is: " + tmp_x(2))

    end
    %%%%

    
    
    vx_vals = AeqPolyDerivative(tmp_x, n_order, 1);
    poly_v_cx = flip(vx_vals);

    vy_vals = AeqPolyDerivative(tmp_y, n_order, 1);
    poly_v_cy = flip(vy_vals);

    %sum_vx = sum(poly_v_cx);
    %sum_vy = sum(poly_v_cy);
    %disp("sum vx is: " + sum_vx);
    %disp("sum vy is: " + sum_vy);
    %disp("sum of vx_vals is " + sum(vx_vals));

    % already checked:
    % here the sum(vx_vals) is equal to polyval(poly_v_cx, ts(i+1));
    disp("ts interval is " + ts(i+1));
    disp("vx1_e is : " + polyval(poly_v_cx, ts(i+1)));
    disp("vy1_e is : " + polyval(poly_v_cy, ts(i+1)));

    %disp("pxe of seg is : " + polyval(poly_cx,1));
    %disp("pye of seg is : " + polyval(poly_cy,1));
    p_coef_x = tmp_x;
    p_coef_y = tmp_y;



    disp("next seg: ");
    disp("");
end

 
for i = 1:n_seg
    disp("play n_seg with i " + i);
    X_n = X_n_n(i,:);
    Y_n = Y_n_n(i,:);
    plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
    hold on
    scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));
    pause(0.1)
    
end



function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
    disp("size of poly_coef is : " + size(poly_coef));
end