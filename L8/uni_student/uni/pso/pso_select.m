function global_best = pso_select(theta,omega,v_ini,p0,last_theta,last_v)
R=[cos(theta) -sin(theta);
   sin(theta)  cos(theta)];
%initialize the particles
N=10;
batch=10;
%theta,v_end,v_theta,v_vend,best_theta,best_v,best_cost
P=zeros(N,7);
global_best = zeros(1,3);
global_best(1) = last_theta-theta;
global_best(2) = last_v;
global_best(3) = evaluate(R,omega,p0,last_theta-theta,last_theta-theta,v_ini,last_v);
for i=1:N
    P(i,1)=(rand-0.5)*1.8*pi;
    P(i,2)=(rand-0.5)*4+2;
    P(i,3)=rand-0.5;
    P(i,4)=rand-0.5;
    P(i,5)=P(i,1);
    P(i,6)=P(i,2);
    P(i,7)=inf;
end


k1 = 0.3;
k2 = 0.2;
w= 0.5

for j=1:batch
    for i=1:N
        w = 0.95-(0.95-0.4)/batch*j;
        if (j~=1)
            %update the particle position for the i'th particle
            %--------------------------------------------------------------
            % To be finished by the student:
            %--------------------------------------------------------------
            
            %1. update theta and v for i'th particle
            %delta_theta = 0.5 * (P(i,5) - P(i,1)) + 0.5* (global_best(1) - P(i,1))
            %delta_v_x = cos(P(i,1))
            %delta_v_y = 
            %%P(i,1) = P(i,1) + delta_theta;
            %P(i,2) = P(i,2) + delta_v;
            %
            %%2. update p0 
            %theta_n = P(i,1);
            %v_c = P(i,2);
            %dt = 0.02;
            %p0[1] = p0[1]+dt*v_c*cos(theta_n);
            %p0[2] = p0[2]+dt*v_c*sin(theta_n);
            
            %pos using P(i,1) and P(i,2)
            [r_x, r_y]  = compute_pos_with_target_theta_v(p0, theta, v_ini, omega, P(i,1), P(i,2) )
            curr_p_x = r_x;
            curr_p_y = r_y;

            [r_x, r_y]  = compute_pos_with_target_theta_v(p0, theta, v_ini, omega, P(i,5), P(i,6) )
            pbest_p_x = r_x;
            pbest_p_y = r_y;

            [r_x, r_y]  = compute_pos_with_target_theta_v(p0, theta, v_ini, omega, global_best(1), global_best(2))
            gbest_p_x = r_x;
            gbest_p_y = r_y;

            % current velocity
            delta_v_x = P(i,2)*cos(P(i,1));
            delta_v_y = P(i,2)*sin(P(i,1));

            delta_v_x = w * delta_v_x + k1 * rand * (pbest_p_x - curr_p_x) + k2 * rand * (gbest_p_x - curr_p_x);
            delta_v_y = w * delta_v_y + k1 * rand * (pbest_p_y - curr_p_y) + k2 * rand * (gbest_p_y - curr_p_y);

            f_v = sqrt(delta_v_x^2 + delta_v_y^2);
            f_theta = acos(delta_v_x / f_v);
            
            P(i,1) = f_theta;
            P(i,2) = f_v;


        end
        %evaluate the particles
        cost = evaluate(R,omega,p0,P(i,1),last_theta-theta,v_ini,P(i,2));
        
        %update the local best
        if cost < P(i,7)
            P(i,7) = cost;
            P(i,5)=P(i,1);
            P(i,6)=P(i,2);
        end
        
        %update the global best
        if cost<global_best(3)
            global_best(3)=cost;
            global_best(1)=P(i,1);
            global_best(2)=P(i,2);
        end
    end
end
global_best(1)=global_best(1)+theta;
end