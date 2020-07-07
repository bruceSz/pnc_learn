function point_list = ellipse_sample(start, goal, max_num, cbest)
    
    count = 0;
    
    cmin=norm(goal-start);
   
    %cbest=20  + cmin;
   
    x_center = [(start + goal)/2, 0];
    disp("center of start and goal is : " + x_center)
    while cbest>=cmin
        x_center=[(start+goal)/2,0];
        x_center=x_center';
        a_1=[(goal(1)-start(1))/cmin;(goal(2)-start(2))/cmin;0];
        id_t=[1,0,0];
        M=a_1*id_t;
        [U,S,Vh]=svd(M);
        C=(U*diag([1,1,det(U)*det(Vh')]))*(Vh);
        r=[cbest/2,sqrt(cbest.^2-cmin.^2)/2,sqrt(cbest.^2-cmin.^2)/2];
        L=diag(r);
        
        a=rand();
        b=rand();
        if b<a
            tmp=b;
            b=a;
            a=tmp;
        end
        x_ball=[b*cos(2*pi*a/b);b*sin(2*pi*a/b);0];
        randpoint=C*L*x_ball+x_center;
        count = count + 1;
        %disp("randpoint is :" + randpoint(1,1))
        x = [randpoint(1,1), randpoint(2,1)];
        point_list.v(count).x = x(1);
        point_list.v(count).y = x(2);
        if count > max_num
            break;
        end
           
       
        cbest=cbest-rand()*0.05;
    end
end