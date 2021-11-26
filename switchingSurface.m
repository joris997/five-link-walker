function [value, isterminal,direction] = switchingSurface(t,x)
    global step downstep failure
    
    p = loadParams;
    
    xy = pJoints(x(1:5),p);
    if step == 3 % should be 3 to be in line with ws_Controller_downstep.mat
        value = [xy(2,end) + downstep]; % is y loc of swing foot
    else
        value = [xy(2,end)];
    end
    isterminal = [1];
    direction = [-1];                   % from positive y to negative y

    % add escape upon failure in optimization
    if failure
        value = [value 0];
    else
        value = [value 1];
    end
    isterminal = [isterminal 1];
    direction = [direction 0];
end

