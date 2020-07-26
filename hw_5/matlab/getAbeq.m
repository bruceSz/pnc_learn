function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    coef_n = n_order + 1;
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    % 
    % 1 beq_start is the start_cond
    % 2 Aeq(1,1) is the p0 of seg_1 here, set it to start_cond(1) 
    % 3 v,a,j all set to 0, when T is 0.
    beq_start(1,1) = start_cond(1,1); 
    
    Aeq_start(1,1) = 1; % p 
    Aeq_start(2,2) = 1; % v
    Aeq_start(3,3) = 1; % a
    Aeq_start(4,4) = 1; % j
    

    %
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %
    % beq_end  is the end_cond
    % 
    %
    ts_end = ts(n_seg);
    vals = getAeqPoly(n_order, ts_end);

    beq_end(1,1) = end_cond(1,1);

    end_idx = n_all_poly - 7;
    Aeq_end(1,end_idx:end_idx + 7) = [1 vals];
    Aeq_end(2,end_idx:end_idx + 7) = AeqPolyDerivative(Aeq_end(1,end_idx:end_idx + 7), n_order, ts_end);
    Aeq_end(3,end_idx:end_idx + 7) = AeqPolyDerivative(Aeq_end(2,end_idx:end_idx + 7), n_order, ts_end);
    Aeq_end(4,end_idx:end_idx + 7) = AeqPolyDerivative(Aeq_end(3,end_idx:end_idx + 7), n_order, ts_end);

    
    %#####################################################
    % position constrain in all middle waypoints
    disp("all poly is : " + n_all_poly);
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    %
    %
    %
    % add for loop here. exclude the end_cond.
    % waypoints from 1 to n_seg-1.
    for i = 1:n_seg - 1
        % 
        % 
        skip = (i-1) * coef_n;
        % here reuse the vals 
        Aeq_wp(i,skip+1:skip + coef_n) = [1 vals];
        beq_wp(i, 1) = waypoints(i+1);
    end
    
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %
    %
    %
    %
    for i = 1:n_seg - 1
        f_skip = (i - 1) * coef_n;
        n_skip = (i) * coef_n;
        Aeq_con_p(i,f_skip + 1 : f_skip + coef_n)  = [1 vals];        
        % for next_seg, t=0 when only the first 
        %Aeq_con_p(i,n_skip + 1:n_skip+8) = [0 -1 0 0 0 0 0 0 ];
        Aeq_con_p(i,n_skip + 1: n_skip + 8) = [ -1 0 0 0 0 0 0 0];
    end
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
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
        v_vals = AeqPolyDerivative(tmp,n_order, 1);
        Aeq_con_v(i,f_skip +1:f_skip+coef_n ) = v_vals;
        Aeq_con_v(i, n_skip+1:n_skip+8) = [ 0 -1 0 0 0 0 0 0];

    end

    disp("size of Aeq_con_v is : row: " + size(Aeq_con_v, 1) + ": cols: " + size(Aeq_con_v, 2));

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    disp("size of Aeq_con_v is : row: " + size(Aeq_con_a, 1) + ": cols: " + size(Aeq_con_a, 2));
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
        v_vals = AeqPolyDerivative(tmp,n_order, 1);
        a_vals = AeqPolyDerivative(v_vals, n_order, 1);

        Aeq_con_v(i, f_skip+1:f_skip+coef_n  ) = a_vals;
        Aeq_con_v(i, n_skip + 3) = -2;
    end
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    %
    %
    %
    %
    for i=1:n_seg - 1
        f_skip = (i-1) * coef_n;
        n_skip = (i) * coef_n;
        tmp = [1 vals ];
        v_vals = AeqPolyDerivative(tmp, n_order, 1);
        a_vals = AeqPolyDerivative(v_vals, n_order, 1);
        j_vals = AeqPolyDerivative(a_vals, n_order, 1);
        Aeq_con_j(i, f_skip + 1:f_skip + coef_n) =j_vals;
        Aeq_con_j(i, n_skip + 4) = -6;
    end
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v];% Aeq_con_a];% Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v];% beq_con_a];% beq_con_j];

    disp("size of Aeq_start: " + size(Aeq_start));
    disp("size of Aeq_end: " + size(Aeq_end));
    disp("size of Aeq_wp: " + size(Aeq_wp));
    disp("size of Aeq_con: " + size(Aeq_con));

    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    disp("size of beq_start: " + size(beq_start));
    disp("size of beq_end: " + size(beq_end));
    disp("size of beq_wp: " + size(beq_wp));
    disp("size of beq_con: " + size(beq_con));

    beq = [beq_start; beq_end; beq_wp; beq_con];
end





