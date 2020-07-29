function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    %
    %
    %
    %
    %

    %
    % for selection matrix C:
    %

    % fix:
    % start cond d00, d01, d02, d03
    % wp1: dt0 == d00'
    % wp2:
    % .
    % .
    % .
    % wpn:
    % end cond dt0, dt1, dt2, dt3

    % free:
    % dt1, dt2, dt3



    % selection M:
    % d00 d01 d02 d03
    % dt0 dt1 dt2 dt3
    % .
    % .
    % n

    Ct = zeros(4 * 2 * n_seg, 4*2 + (n_seg-1)*4 );
    % (n_seg-1) waypoints.
    % beg and end.
    
    % n_seg +  7
    fix_num = 4 + 4 + (n_seg - 1);
    % 3*(n_seg - 1)
    free_num = (n_seg-1) * 3;
    for i =1:n_seg

        % dealing with four d of this segment.
        beg_i_n = 4;
        %tmp_b = zeros(4,4);
        if i == 1
            for j = 1:beg_i_n               
                Ct(j,j) = 1;
            end
        else
            for j = 1:beg_i_n
                row_idx = (i-1) * 8 + j;
                if j == 1
                    p_idx = i-1;
                    col_idx = p_idx  + 4;
                    %row_idx = 
                    Ct(row_idx, col_idx) = 1;
                else
                    col_idx = fix_num + (i-2)*3 + (j-1);
                    %row_idx = (i-1) * 8 + j;
                    Ct(row_idx, col_idx) = 1;
                end
            end
            
        end

        % dealing with four end d of this segment.
        if i == n_seg
            %
            % for 
            %
            for j = 1:4
                % map to the end 4 elements for fixed variable.
                row_idx = (i-1)*8 + 4 + j;
                col_idx = 4 + (n_seg - 1)  + j;
                Ct(row_idx, col_idx) = 1;
            end
        else
            for j = 1:4
                row_idx = (i-1)* 8 + 4 + j;
                if j == 1
                    col_idx= i + 4;
                    Ct(row_idx, col_idx) = 1;
                else
                    col_idx = fix_num + (i-1)*3 + (j-1);
                    Ct(row_idx, col_idx) = 1;
                end
            end

        end
        

    end

end