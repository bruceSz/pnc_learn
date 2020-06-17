function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    %genpath(pwd)
    disp("starting the a star search.")
    disp("MAX X is "  + MAX_X + " MAX Y is:  " + MAX_Y )   
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    disp("target x: " + xTarget + "; target y: " + yTarget)
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    disp(" a star pwd is: " + pwd)
    disp(" a star fileparts: "  + fileparts(pwd))
    disp(" a star gen path is : " + genpath([fileparts(pwd),filesep, 'code']))
    % it seems that whether we have this path , main.m can always run.
    %addpath(genpath([fileparts(pwd),filesep, 'code']))
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    scatter(xNode-0.5, yNode-0.5, 'g');
    hold on
    disp("open list after insert_open the start node." + size(OPEN))
    disp( "open_count: "  + OPEN_COUNT)
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
    
    % expand all node from start.
    expand_node = expand_array(OPEN(1,2),OPEN(1,3),path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y)
    fprintf("expand from x:" + OPEN(1,2) + "; y: " + OPEN(1,3))
    expand_size = size(expand_node,1)
    for i=1:expand_size
        x_tmp = expand_node(i, 1)
        y_tmp = expand_node(i, 2)
        
        hn = expand_node(i, 3);
        gn = expand_node(i, 4);
        fn = expand_node(i, 5);
        OPEN_COUNT = OPEN_COUNT + 1
        OPEN(OPEN_COUNT, :) = insert_open(x_tmp, y_tmp, xNode, yNode, hn, gn, fn )
        scatter(x_tmp-0.5, y_tmp-0.5, 'g');
            hold on
        
    end
%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    index = min_fn(OPEN,OPEN_COUNT, xTarget, yTarget)
    while(index ~= -1) %you have to dicide the Conditions for while loop exit 
        
     %
     %finish the while loop
     %
     curr_x = OPEN(index, 2);
     curr_y = OPEN(index, 3);
     path_cost = OPEN(index, 7)
     OPEN(index,1) = 0
     CLOSED_COUNT=CLOSED_COUNT+1;
     CLOSED(CLOSED_COUNT,1)=curr_x;
     CLOSED(CLOSED_COUNT,2)=curr_y;
     
     if (curr_x == xTarget && curr_y == yTarget)
        disp("found target at " + index) 
         break
     end
     
     expand_node = expand_array(curr_x,curr_y,path_cost, xTarget, yTarget , CLOSED, MAX_X, MAX_Y)
     expand_size = size(expand_node,1)
     for i=1:expand_size
        x_tmp = expand_node(i, 1);
        y_tmp = expand_node(i, 2);
        hn = expand_node(i, 3);
        gn = expand_node(i, 4);
        fn = expand_node(i, 5);
        % check and update if the x_tmp is already in open list.
        
        exist_index = node_index(OPEN, x_tmp, y_tmp)
        if (exist_index == -1)
            
            OPEN_COUNT = OPEN_COUNT + 1
            OPEN(OPEN_COUNT, :) = insert_open(x_tmp, y_tmp, curr_x, curr_y, hn, gn, fn )
            scatter(x_tmp-0.5, y_tmp-0.5, 'g');
            hold on
            
        else
            old_path_cost = OPEN(exist_index, 7)
            if gn < old_path_cost
                % update gn and update parent_child relationship
                OPEN(exist_index, 7) = gn
                OPEN(exist_index, 4) = curr_x
                OPEN(exist_index, 5) = curr_y
            end
        end
        
     end
     
     index = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget)
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    
    
   path = [];
   
   path_index = 1;
   % met the target.
   if (index ~= -1)
       new_row = [1,2]
       new_row(1,1) = OPEN(index, 2)
       new_row(1,2) = OPEN(index, 3)
       path(path_index,:) = new_row
       path_index = path_index + 1
       
       p_x = OPEN(index, 4)
       p_y = OPEN(index, 5)
       
       index = node_index(OPEN, p_x, p_y)
       while(p_x~=xStart || p_y ~= yStart)
           disp("adding path point.")
           new_row = [1,2];
           new_row(1,1) = OPEN(index, 2);
           new_row(1,2) = OPEN(index, 3);
           path(path_index,:) = new_row
           path_index = path_index + 1
           p_x = OPEN(index, 4);
           p_y = OPEN(index, 5);
           index = node_index(OPEN, p_x, p_y)
       end
       new_row = [1,2];
       new_row(1,1) = p_x;
       new_row(1,2) = p_y;
       path(path_index,:) = new_row
   else
       disp("not found a valid path.")
        
   end
   % loop from target to start, chained by parent-children link.
end


