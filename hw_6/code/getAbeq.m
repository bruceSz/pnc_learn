function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    coef_n = n_order + 1;
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    beq_start = zeros(3, 1);
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    %Aeq_end = [];
    %beq_end = [];

    beq_start(1,1) = start_cond(1,1); 
    
    Aeq_start(1,1) = 1; % p 
    Aeq_start(2,2) = 1; % v
    Aeq_start(3,3) = 2; % a
    %Aeq_start(4,4) = 6; % j
    

    %
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    beq_end = zeros(3, 1);

    ts_end = ts(n_seg);
    vals = getAeqPoly(n_order, ts_end);

    beq_end(1,1) = end_cond(1,1);

    end_idx = n_all_poly - n_order;

    Aeq_end(1,end_idx:end_idx + n_order) = [1 vals];
    % for 1 1 1 1 1 ... , high to low and low to high are the same.
    xx = PolyDerivative(Aeq_end(1,end_idx: end_idx + n_order), n_order, ts_end);
    
    xx_l2h = flip(xx);
    
    Aeq_end(2,end_idx:end_idx + n_order) = xx_l2h;%PolyDerivative(Aeq_end(1,end_idx:end_idx + 7), n_order, ts_end);

    xx = PolyDerivative(flip(Aeq_end(2, end_idx: end_idx + n_order)), n_order-1, ts_end);
    xx_l2h = flip(xx);
    Aeq_end(3,end_idx:end_idx + n_order) = xx_l2h;%PolyDerivative(Aeq_end(2,end_idx:end_idx + 7), n_order-1, ts_end);

    %xx = PolyDerivative(flip(Aeq_end(3, end_idx: end_idx + n_order)), n_order-2, ts_end);
    %disp("xx is: " + xx);
    %xx_l2h = flip(xx);
    %disp("flip xx is: " + xx_l2h);
    %Aeq_end(4,end_idx:end_idx + n_order) = xx_l2h;%PolyDerivative(Aeq_end(3,end_idx:end_idx + 7), n_order-1, ts_end);
    
    

    %#####################################################
    % STEP 2.1 position continuity constrain between 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);

    for i = 1:n_seg - 1
        f_skip = (i - 1) * coef_n;
        n_skip = (i) * coef_n;
        Aeq_con_p(i,f_skip + 1 : f_skip + coef_n)  = [1 vals];        
        % for next_seg, t=0 when only the first 
        %Aeq_con_p(i,n_skip + 1:n_skip+8) = [0 -1 0 0 0 0 0 0 ];
        Aeq_con_p(i,n_skip + 1: n_skip + 8) = [ -1 0 0 0 0 0 0 0];
    end

    %#####################################################
    % 2.2 velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    %
    %
    %
    %
    for i=1:n_seg - 1
        f_skip = (i-1) * coef_n;
        n_skip = (i) * coef_n;
        tmp = [1 vals];
        tmp = flip(tmp);
        v_vals = PolyDerivative(tmp,n_order, 1);
        %disp("v_vals is: "+ v_vals);
        Aeq_con_v(i,f_skip +1:f_skip+coef_n ) = flip(v_vals);
        Aeq_con_v(i, n_skip+1:n_skip+8) = [ 0 -1 0 0 0 0 0 0];

    end

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    %disp("size of Aeq_con_v is : row: " + size(Aeq_con_a, 1) + ": cols: " + size(Aeq_con_a, 2));
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    %
    %
    %
    %
    for i=1:n_seg - 1
        f_skip = (i -1) * coef_n;
        n_skip = (i) * coef_n;
        tmp = [1 vals];
        % flip to h2l.
        tmp =  flip(tmp);
        v_vals = PolyDerivative(tmp,n_order, 1);
        
        a_vals = PolyDerivative(v_vals, n_order-1, 1);
     
        Aeq_con_a(i, f_skip+1:f_skip+coef_n  ) = flip(a_vals);
        Aeq_con_a(i, n_skip + 3) = -2;
    end

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end