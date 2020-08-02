function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        %
        %
        %
        %
        
        % current time allocated.
        t_interval = ts(k);
        % there are total n_order, and for order i <4 (0-3), the Q_k(i) is zero.
        coef_n = n_order + 1;
        for i_n = 1:coef_n
            % here the 1->p0
            % 
            %Q_k(i) = [];
            if i_n < 5
                % init a zero list and insert into  Q_k.
                tmp_matrix = zeros(1,coef_n);
                tmp_arr = tmp_matrix(1,:);
                %Q_k(qk_count) = tmp_arr;
                Q_k(i_n,:) = tmp_arr;
                %qk_count  = qk_count +1;
            else
                tmp_arr = [];
                %tmp_arr_count = 1
                i = i_n - 1;

                for jj_n = 1:coef_n
                    if jj_n < 5
                        item = 0;
                    else
                        jj = jj_n - 1;
                        item = i * (i-1) * (i-2) * (i-3) * jj * (jj-1) * (jj - 2) * (jj-3) / (i + jj - 7)  * t_interval;
                    end
                    tmp_arr(jj_n) = item;
                    
                end
                Q_k(i_n,: ) = tmp_arr;
                   
            end 
            
        end
        
        Q = blkdiag(Q, Q_k);
    end
end