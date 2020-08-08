function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    coef_n = n_order + 1;
    %#####################################################
    % STEP 3.2.1 p constraint (safe constraint).
    % for all control point, need to set them in between the corridor_range.
    disp("n_all_poly is : " + n_all_poly);
    Aieq_p = zeros(coef_n*n_seg*2 + (n_seg-1)*2, n_all_poly);
    disp("size of Aieq_p is : " + size(Aieq_p));
    disp("coef_n is : " + coef_n);
    disp("n_seg is : " + n_seg);
    bieq_p = zeros(coef_n*n_seg*2 + (n_seg-1)*2, 1);

    k = 1;
    for i=1:n_seg
        skip = (i-1) * coef_n;
        range = corridor_range(i,:);
        s_val = range(1);
        e_val = range(2);
        disp(i+" s_val is : " + s_val);
        disp(i+" e_val is : " + e_val);
        % for all control point.
        % c <= p_e
        for j = 1: coef_n
            Aieq_p(k,skip+j) = 1;
            bieq_p(k) = e_val;
            k=k+1;
        end

        % c >= p_s -> -c <= -p_s
        for j = 1:coef_n
            Aieq_p(k,skip+j) = -1;
            bieq_p(k) = -s_val; 
            k = k+1;
        end
    end

    disp(" after set safe constraint, k is: " + k);

    for i = 1:n_seg
        skip = (i-1) * coef_n;
        % end contol point of this segment can be in between next corridor_range.
        if  i<n_seg
            next_range = corridor_range(i+1,:);
            next_s = next_range(1);
            next_e = next_range(2);
            Aieq_p(k,skip+coef_n) = 1;
            bieq_p(k) = next_e;
            k = k+1;

            Aieq_p(k,skip+coef_n) = -1;
            bieq_p(k) = -next_s;
            k = k+1;

        end
    end

    for i = 1:n_seg
        skip = (i-1) * coef_n;

        % start control point of this segment can be in between prev corridor_range.
        if i > 1
            prev_range = corridor_range(i-1,:);
            prev_s = prev_range(1);
            prev_e = prev_range(2);
            Aieq_p(k,skip+1)  = 1;
            bieq_p(k) = prev_e;
            k = k+1;

            Aieq_p(k,skip+1)  = -1;
            bieq_p(k) = -prev_s;
            k = k+1;
        end

    end

    %#####################################################
    % STEP 3.2.2 v constraint  
    % th ev is the first order derivative of ori b-curve. 
    Aieq_v = zeros(n_seg*(coef_n-1)*2,n_all_poly);
    bieq_v = zeros(n_seg*(coef_n-1)*2,1);
    k = 1;
    for i = 1:n_seg
        skip = (i-1)* coef_n;
        % according to formular cn' = n * (cn+1 - cn);
        for j = 1:coef_n-1
            n_idx = skip + j+1;
            idx = skip + j;
            Aieq_v(k,n_idx) = n_order ;
            Aieq_v(k,idx) = n_order * (-1) ;
            bieq_v(k) = v_max;
            k = k+1;
        end
%
        for m = 1:coef_n -1
        %
             n_idx = skip + m + 1;
             idx = skip + m;
             Aieq_v(k,n_idx) = n_order * (-1);
             Aieq_v(k,idx) = n_order;
             bieq_v(k) = v_max;
            k = k+1
        end
    end


    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = zeros(n_seg*(coef_n-2)*2, n_all_poly);
    bieq_a = zeros(n_seg*(coef_n-2)*2,1);

    k = 1;
    %for i = 1:n_seg
    %    skip = (i-1)*coef_n;
    %    for j = 1:coef_n-2
    %        nn_idx = skip + j + 2;
    %        n_idx = skip + j + 1;
    %        idx = skip + j;
    %        Aieq_a(k,nn_idx) = n_order* (n_order-1);
    %        Aieq_a(k,n_idx) = n_order*(n_order-1) * (-2) ;
    %        Aieq_a(k,idx) = n_order * (n_order -1 );
    %        bieq_v(k) = a_max;
    %    end
%
    %    for j = 1:coef_n-2
    %        nn_idx = skip + j + 2;
    %        n_idx = skip + j + 1;
    %        idx = skip + j;
    %        Aieq_a(k,nn_idx) = -1* n_order* (n_order-1);
    %        Aieq_a(k,n_idx) = -1* n_order*(n_order-1) * (-2) ;
    %        Aieq_a(k,idx) = -1*  n_order * (n_order -1 );
    %        bieq_v(k) = -a_max;
    %    end
%
    %end
    disp("size of Aieq_p: " + size(Aieq_p));
    disp("size of  Aieq_v: " + size(Aieq_v));
    disp("size of Aieq_a: " + size(Aieq_a));
    
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    %Aieq = Aieq_p;
    %bieq = bieq_p;
end