
function rmse_r = rmse(Y)

    n = size(Y, 2);
    temp_sum = 0;
    for k = 1 :  n
        temp_sum = temp_sum + Y(1,k)^2;
    end
    rmse_r = sqrt(temp_sum/n);
end

