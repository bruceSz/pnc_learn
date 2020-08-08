clc;clear;close all
v_max = 400;
a_max = 400;
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];

%% specify the center points of the flight corridor and the region of corridor
path = [50, 50;
       100, 120;
       180, 150;
       250, 80;
       280, 0];
x_length = 100;
y_length = 100;

n_order = 7;   % 8 control points
n_seg = size(path, 1);

corridor = zeros(4, n_seg);
for i = 1:n_seg
    corridor(:, i) = [path(i, 1), path(i, 2), x_length/2, y_length/2]';
end

disp("corridor is : " + corridor);

%% specify ts for each segment
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i,1) = 1;
end

poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor, ts, n_seg, n_order, v_max, a_max);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor, ts, n_seg, n_order, v_max, a_max);


poly_coef_x  = poly_coef_x';
poly_coef_y = poly_coef_y';
disp("size of poly_coef_x is: " + size(poly_coef_x));
disp("size of poly_coef_y is: " + size(poly_coef_y));
%% display the trajectory and cooridor
plot(path(:,1), path(:,2), '*r'); hold on;
for i = 1:n_seg
    plot_rect([corridor(1,i);corridor(2,i)], corridor(3, i), corridor(4,i));hold on;
end
hold on;
x_pos = [];y_pos = [];
idx = 1;

%% #####################################################
% STEP 4: draw bezier curve
start_c_x = [];
start_c_y = [];
start_c_idx = 1;


for k = 1:n_seg
    skip = (k-1)*(n_order + 1);
    tmp_x = poly_coef_x(1, skip+1: skip+n_order+1);
    tmp_y = poly_coef_y(1, skip+1: skip+n_order+1);
    start_c_x(start_c_idx) = tmp_x(1);
    start_c_y(start_c_idx) = tmp_y(1);
    start_c_idx = start_c_idx + 1;

    for t = 0:0.01:1
        x_pos(idx) = 0.0;
        y_pos(idx) = 0.0;
        for i = 0:n_order
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
            
            x_pos(idx) =   x_pos(idx) + basis_p * tmp_x(i+1);
            y_pos(idx) =  y_pos(idx) + basis_p * tmp_y(i+1);
            
             
        end
        idx = idx + 1;
    end
end
plot(x_pos, y_pos , 'Color', [0 1.0 0], 'LineWidth', 2);
scatter(start_c_x, start_c_y,'filled','g');
% plot(...);

function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, n_order, v_max, a_max)
    start_cond = [waypoints(1), 0, 0];
    end_cond   = [waypoints(end), 0, 0];   
    
    %% #####################################################
    % STEP 1: compute Q_0 of c'Q_0c
    [Q, M]  = getQM(n_seg, n_order, ts);
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    disp("size of M is : " + size(M));
    disp("size of Q0 is : " + size(Q_0))
    
    %% #####################################################
    % STEP 2: get Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);

    %
    % equal formular: Aeq * p = beq
    % p = M * c;
    % transform it to Aeq_new * c = beq by replace p with M * C
    % Aeq_new = Aeq * M;
    %
    Aeq = Aeq * M;
    
    %% #####################################################
    % STEP 3: get corridor_range, Aieq and bieq 
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    % you can define corridor_range as [p1_min, p1_max;
    %                                   p2_min, p2_max;
    %                                   ...,
    %                                   pn_min, pn_max ];
    corridor_range = [];
    for i = 1 : size(corridor,2)
        c_box = corridor(:, i);
        center = c_box(axis);
        low_b = center - c_box(3);
        upper_b = center + c_box(4);
        corridor_range(i,:) = [low_b, upper_b];
    end
    
    % STEP 3.2: get Aieq and bieq
    [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max);
    
    f = zeros(size(Q_0,1),1);
    poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

function plot_rect(center, x_r, y_r)
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];    
    plot(a(1,:),a(2,:),'b');
end