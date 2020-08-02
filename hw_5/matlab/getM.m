function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = [];
        t_interval = ts(k);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        %
        %
        %
        %
        % Mj is one matrix generalized from M in page.45 in lecture5.
        for i = 1:8
            % change order (low to high order).
            if i == 1
                tmp_k = zeros(1,n_order + 1);
                tmp_k(1, 1) = 1;
            end

            if i == 2
                tmp_k = zeros(1,n_order + 1);
                tmp_k(1, 2) = 1;
            end

            if i == 3
                tmp_k = zeros(1,n_order + 1);
                tmp_k(1, 3) = 2;
            end

            if i == 4
                tmp_k = zeros(1,n_order + 1);
                tmp_k(1, 4) = 6;
            end
            
            if i == 5
                vals = getAeqPoly(n_order, t_interval);
                % flip to make seq from high order to low order.
               % vals = flip(vals);
                tmp_k = [1 vals ];
            end

            if i == 6
                vals = getAeqPoly(n_order, t_interval);
                % flip to make seq from high order to low order.
                vals = flip(vals);
                new_coef = PolyDerivative(vals, n_order, t_interval);
                % flip back to(from low to high)
                new_coef = flip(new_coef);
                tmp_k = [0 new_coef ];

            end

            if i == 7
                vals = getAeqPoly(n_order, t_interval);
                % flip to make seq from high order to low order.
                vals = flip(vals);

                coef_d1 = PolyDerivative(vals, n_order, t_interval);
                coef_d2 = PolyDerivative(coef_d1, n_order-1, t_interval);
                % flip back to (from low to high).
                coef_d2 = flip(coef_d2);
                tmp_k = [0 coef_d2];
            end

            if i==8
                vals = getAeqPoly(n_order, t_interval);
                vals = flip(vals);
                coef_d1 = PolyDerivative(vals, n_order, t_interval);
                coef_d2 = PolyDerivative(coef_d1, n_order-1, t_interval);
                coef_d3 = PolyDerivative(coef_d2, n_order-2, t_interval);
                
                %flip back to from low to high.
                coef_d3  = flip(coef_d3);
                tmp_k = [0 coef_d3];    
            end

            %disp("for row "+ i  + " we assign: " +size(tmp_k,2));
            M_k(i,:) = tmp_k;

        end 
        M = blkdiag(M, M_k);
    end
end