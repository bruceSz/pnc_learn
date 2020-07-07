clc;
clear all; close all;

% refer: https://blog.csdn.net/ljq31446/article/details/79708228
% Informed RRT*: Optimal Incremental Path Planning Focused through an Admissible Ellipsoidal Heuristic

test_llipse_sample()

function flag = test_llipse_sample()
    start=[20,120];goal=[100,80];
    axis([0,200,0,200]);

    plot(start(1), start(2),'o')
    hold on
    grid on
    plot(goal(1), goal(2),'*')
    hold on
    grid on

    cmin=norm(goal-start);
    points = ellipse_sample(start, goal, 100, cmin+2)
    disp("size of points: " + size(points.v,2))
    for i = 1:size(points.v,2)
        disp("points i is: x:" + points.v(i).x + " ; y: " + points.v(i).y)
        tmp_p = [points.v(i).x, points.v(i).y]
        
        scatter(tmp_p(1), tmp_p(2),  'b');
        %plot(tmp_p(1), tmp_p(2), '.')
        %hold on 
            %grid on
    end

end


