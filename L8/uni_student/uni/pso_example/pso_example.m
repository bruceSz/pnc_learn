c1 = 1;
c2 = 1;
Dim = 20;
size = 30;
Tmax_threshold = 600;
Vmax_threshold = 200;

F_n = 3;
Fun_ub = 800;
Fun_lb = -800;

Position=zeros(Dim, size);
Vel = zeros(Dim, size);

Vmax(1:Dim) = Vmax_threshold;
Vmin(1:Dim) = -Vmax_threshold;
Xmax(1:Dim) = Fun_ub;
Xmin(1:Dim) = -Fun_lb;

[Position, Vel] = Init_pos_vel(Dim, size, Xmax, Xmin, Vmax, Vmin);
Lbest_pos = Position;
Gbest_pos = zeros(Dim, 1);


% the first time , compute all fit_score.
for j = 1:size
    pos = Position(:,j);
    fz(j) = Fit_func(pos, F_n, Dim);
end

% compute g best_score. I is the index.
[Gbest_F, I] = min(fz);

Gbest_pos = Position(:,I);

% Tmax max iteration.
for itr = 1:Tmax_threshold
    time(itr)  = itr;
    w = 1;
    r1 = rand(1);
    r2 = rand(1);
    
    for i = 1:size
        % for each dimension, update velocity using the velocity update
        % formular
        Vel(:,i) = w * Vel(:,i)  + c1 * r1 * (Lbest_pos(:,i)-Position(:,i))  + c2 * r2 * (Gbest_pos - Position(:,i));
    end
    % vel process
    for i = 1: size
        for row = 1:Dim
            if Vel(row,i)  > Vmax(row)
                Vel(row, i)  = Vmax(row);
            elseif Vel(row, i) < Vmin(row)
                Vel(row,i) = Vmin(row);
            else
            end
        end
        
    end
    
    Position = Position + Vel;
    
    % boundary process
    for i = 1:size
        for row = 1:Dim
            if Position(row,i)>Xmax(row)
                Position(row,i) = Xmax(row)
            elseif Position(row, i) < Xmin(row)
                Position(row, i) = Xmin(row);
            else
            end
            
        end
    end
    
    % 
    for j = 1:size
        P_pos = Position(:,j)';
        fit_p(j) = Fit_func(P_pos, F_n, Dim)
        
        if fit_p(j) < fz(j)
            Lbest_pos(:,j) = Position(:,j)
            fz(j) = Fit_func(j);
        end
        
        if fit_p(j) < Gbest_pos
            Gbest_pos = fit_p(j);
        end
        
    end
    
    [Gbest_F_new,I] = min(fz);
    Best_fitness(itr) = Gbest_F_new;
    Gbest_pos = Lbest_pos(:,I);
end

plot(time,Best_fitness);
xlabel('迭代的次数');ylabel('适应度值F');

