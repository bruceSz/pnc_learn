%***************************************
%Author: Song Zhang
%Date: 2019-10-15
%***************************************

%% 流程初始化
clc
clear all; close all;
x_I=50; y_I=50;           % 设置初始点
start = [x_I, y_I]
x_G=700; y_G=700;       % 设置目标点
goal = [x_G, y_G]
Thr=50;                 % 设置目标点阈值
Delta= 30;              % 设置扩展步长

%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % 从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     %
T.v(1).all_dist = 0;
%% 开始构建树——作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%地图x轴长度
yL=size(Imp,2);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count=1;
total_iter = 600;

%Snapshot cost:  Met node around goal with iter: 283


points = ellipse_sample(start, goal, total_iter)
all_samples = size(points.v,2)

for iter = 1:all_samples
%for iter = 1:total_iter
   
    %Step 1: 在地图中随机采样一个点x_rand
    %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标
    x_rand = [];
    x_rand(1) = points.v(iter).x;
    x_rand(2) = points.v(iter).y;
    % nearest_node = 
    
    % x_near=[];
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    %提示：x_near已经在树T里
    

    [x_near, x_near_idx] = findNearest(T, count, x_rand(1), x_rand(2));
    %x_near = node_dis(1)
    %dis = node_dis(2)
    disp("nearest node is: x:" +x_near(1) + "; y:"+ x_near(2))

    x_new=[];
    %Step 3: 扩展得到x_new节点
    %提示：注意使用扩展步长Delta
    x_new = try_rand_node([x_near(1), x_near(2)], x_rand, Delta);


    %if dis < Delta
    %    x_new(1) = x_near
    %else
    %    
    %end

    %检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
        disp("collision failed ignore x_new.")
        continue;
    end
    count=count+1;

    %scatter(x_new(1), x_new(2),  'g');
    

    [flag, new_T] = insert_into_tree(count, T, x_new, 50);
    
    %Step 4: 将x_new插入树T 
    %提示：新节点x_new的父节点是x_near
    if flag == false
        disp("find another rand, inset failed.")
        return;
    end
    T = new_T;

    
    %Step 5:检查是否到达目标点附近 
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    if computeDistance(x_new(1),x_new(2), x_G, y_G ) <= Thr
        disp("Met node around goal with iter: " + iter)
        line([x_near(1),x_new(1)], [x_near(2),   x_new(2)],'Color','b',  'Linewidth', 1);
        hold on
        break;
    end
    
   %Step 6:将x_near和x_new之间的路径画出来
   %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
   %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来
   line([x_near(1),x_new(1)] , [x_near(2), x_new(2)], 'Color',  'b', 'Linewidth', 1);
   hold on
   
   pause(0.2); %暂停0.1s，使得RRT扩展过程容易观察
end

%% 路径已经找到，反向查询
if iter < all_samples

    % count is the last node added to T.
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(count).x;
    path.pos(2).y = T.v(count).y;
    pathIndex = T.v(count).indPrev; % 终点加入路径
    j=0;
    while pathIndex ~= 1
        %pathIndex = cast(pathIndex, 'int32')
        %disp("x is : " + T.v(pathIndex).x)
        %disp("pathIndex is :" + pathIndex)
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点

    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        line([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'Color','r', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


function [nearest_nodexxx, nearest_node_idx] =findNearest(T, t_size,x, y)
    ret_n_node = T.v(1);
    node_index = 1;
    disp("T has size: " + size(T.v)  + " t_size is : " + t_size)
    min_distance = inf;
    for i = 1:t_size
        % iterate all nodes in the T and compute distance
        tmp_node = T.v(i);
        
        dis = computeDistance(x, y, tmp_node.x, tmp_node.y);
        if dis < min_distance
            min_distance = dis;
            node_index = i;
            ret_n_node = tmp_node;
        end
    end
    
    nearest_nodexxx = [ret_n_node.x ret_n_node.y];

    nearest_node_idx = node_index;
    %return [n_node min_dis]

end


function new_node = try_rand_node(near_node, rand_node, threshold)
    dis = computeDistance(near_node(1),near_node(2), rand_node(1), rand_node(2));
    ret = [ 0.0 0.0];
    if dis >= threshold
        % for both dx and dy , proportional to threshold/dis
        ret(1) = near_node(1) +  (rand_node(1) - near_node(1)) * threshold / dis;
        ret(2) = near_node(2) + (rand_node(2) - near_node(2)) * threshold / dis;
    else
        ret(1) = rand_node(1);
        ret(2) = rand_node(2);
    end

    new_node = ret;

end

function [flag, new_T] = insert_into_tree(count, T, x_new, radius)
    % 1 find near_node_list using r.
    % 2 compute the target parrent.
    % 3 insert into the T.
    % 4 rewire the T
    
    % 1
    disp("begin insert into the tree")
    near_list  = find_near_node_within(count-1 , T, x_new, radius);
    

    disp("near node count within radius size: " +  (near_list))
    %near_list = near_list_p(1) 
    %near_count = near_list_p(2)

    % 2
    [target_node, target_idx] = target_parrent(T, near_list, x_new);
    disp("find a target_node with idx: " + target_idx)
    %target_node = target_node_p(1)
    %target_idx = target_node_p(2)
    if target_idx == -1
        flag = false
        return
    end

    % 3
    T.v(count).x = x_new(1);         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = target_node.x;     % 起始节点的父节点仍然是其本身
    T.v(count).yPrev = target_node.y;
    T.v(count).dist=computeDistance(target_node.x, target_node.y, x_new(1), x_new(2));          % 从父节点到该节点的距离，这里可取欧氏距离
    T.v(count).indPrev = target_idx;     % index of parent?
    T.v(count).all_dist = target_node.all_dist + T.v(count).dist;

    %4
    rewire(near_list, T, count);
    flag = true;
    disp("after rewire cout : " + count + " T size is : " + size(T.v))
    new_T = T;  

end


function T = rewire(near_list, T, count)
    curr_node = T.v(count);
    near_count = size(near_list, 1)
    for i=1:near_count-1
        near_node = near_list.v(i)
        tmp_dist = computeDistance(near_node.x, near_node.y, curr_node.x, curr_node.y)
        tmp_all_dis = curr_node.all_dist  + tmp_dist
        if tmp_all_dis < near_node.all_dist
            % update the dist, indPrev, xPrev, yPrev
            near_node.xPrev = curr_node.x
            near_node.yPrev = curr_node.y
            near_node.dist = tmp_dist
            near_node.indPrev = count
            near_node.all_dist = tmp_all_dis

        end
    end
end

function [parrent_node, parrent_idx] = target_parrent(T, near_list, x_new)
    near_count = size(near_list)
    if near_count == 0
        disp("Error there is no near node.")
        parrent_idx  = -1;
        % parrent_node empty struct
        parrent_node = struct ;
    end

    %if near_count == 1
    %    disp("Error find a parrent  the radius maybe too big")
    %    parrent_idx  = -1
    %    parrent_node = struct
    %    return
    %    
    %end
    f_index = near_list(1)
    disp("f_index is : " + f_index)
    f_node = T.v(f_index)
    min_path_dis = computeDistance(f_node.x, f_node.y, x_new(1), x_new(2)) + f_node.all_dist;
    min_node = f_node;
    min_idx = f_index;
    disp("near list count is : " + size(near_list) + " provided count is : "+near_count)
    for i=2:near_count
        node_idx = near_list(i)
    
        c_node = T.v(node_idx)
        curr_dis = computeDistance(c_node.x , c_node.y, x_new(1), x_new(2))
        curr_all_dis = curr_dis + c_node.all_dist
        if curr_all_dis < min_path_dis
            min_node = c_node;
            min_path_dis = curr_all_dis;
            min_idx = node_idx
            %parrent_idx = i
        end
    end
    parrent_node = min_node;
    parrent_idx = min_idx;
    
    %all_dis = min_path_dis
end

function near_listxx = find_near_node_within(count, T, x_new, radius)
    %near_listxx;
    near_count = 0;
    node_listxx = []
    %near_listxx.v(near_count) = 
    %curr_node = T.v(near_count);
    %near_listxx(near_count) = curr_node;
    
    %[x_near, x_near_idx] = findNearest(T, count-1, x_new(1), x_new(2))
    %near_listxx(near_count) = x_near
    %near_count = near_count + 1

    for i=1:count
        curr_node = T.v(i);
        dis = computeDistance(curr_node.x, curr_node.y, x_new(1), x_new(2));
        if dis <= radius
            disp("assignment begin with index: " + i);
            near_count = near_count + 1;
            near_listxx(near_count) = i;
            disp("assignment end. count: "  + near_count + " true size: " + near_listxx + "\n");
            
        end
    end
    if near_count == 0
        disp("There is no node with redius of : " + radius)
        %near_listxx = struct
        %near_listxx.v = null
    end
    disp("leaving find_near_node_within: n_c: " + near_count + " true size of node: " + near_listxx)
end

function distance = computeNodeDist(p1, p2)
    distance = sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2)
end

function distance = computeDistance(x1,y1, x2, y2)
    distance =  sqrt((x1-x2)^2 + (y1-y2)^2);
end
