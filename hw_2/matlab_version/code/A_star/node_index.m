function n_index = node_index(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    len = size(OPEN, 1)
    while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
        i=i+1;
        if (i > len)
            break;
        end
    end;
    if (i <= len)
        n_index=i;
    else 
        n_index = -1
    end
end