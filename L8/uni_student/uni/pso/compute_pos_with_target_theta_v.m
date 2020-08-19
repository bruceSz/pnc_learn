function [r_x,r_y] = compute_pos_with_target_theta_v(p0, theta, v_c, omega, theta_target, v_target )
   % copied from pso_test_obstacle.m 
   x = p0(1);
   y = p0(2);

   dt = 0.05;
    
    % Prepare the translational trajectory
    a_star = sign(v_target-v_c)*2;

    if v_target~=v_c
        t_acc = (v_target-v_c)/a_star;
    else
        t_acc = 0;
    end

    [success,thetaL,wL]=second_order_trajectory(omega,theta_target-theta);
    thetaL = thetaL+theta;
    
    % Forward simulation of the vehicle
    for t=dt:dt:0.1
        % First get the angular part
        idx = round(t/dt+1);
        if idx > length(thetaL)
            idx = length(thetaL);
        end
        theta = thetaL(idx);
        omega = wL(idx);
        
        % Second get the translational part
        if t<=t_acc
            v_c = v_c+a_star*dt;
        else
            v_c= v_target;
        end
        
        % Move the vehicle according to a simple model
        x = x+dt*v_c*cos(theta);
        y = y+dt*v_c*sin(theta);
    
    end
    r_x = x;
    r_y = y;

end