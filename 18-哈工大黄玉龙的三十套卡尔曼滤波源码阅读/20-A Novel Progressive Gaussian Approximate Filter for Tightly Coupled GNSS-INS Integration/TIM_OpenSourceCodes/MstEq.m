function y=MstEq(x,GNSS_measurements,est_r_eb_e_old,est_v_eb_e_old,j)

    c = 299792458; % Speed of light in m/s
    omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
    % Skew symmetric matrix of Earth rate
    Omega_ie = Skew_symmetric([0,0,omega_ie]);

    u_as_e_T = zeros(1,3);
    pred_range = 0;
    pred_vel=0;
    
    % Predict approx range 
    delta_r = GNSS_measurements(j,3:5)' - (est_r_eb_e_old-x(7:9,1));
    approx_range = sqrt(delta_r' * delta_r);

    % Calculate frame rotation during signal transit time using (8.36)
    C_e_I = [1, omega_ie * approx_range / c, 0;...
             -omega_ie * approx_range / c, 1, 0;...
             0, 0, 1];

    % Predict pseudo-range using (9.165)
    delta_r = C_e_I *  GNSS_measurements(j,3:5)' - (est_r_eb_e_old-x(7:9,1));
    range = sqrt(delta_r' * delta_r);
    pred_range = range + x(16);    
        
    % Predict line of sight
    u_as_e_T(1:3) = delta_r' / range;

    range_rate = u_as_e_T(1:3) * (GNSS_measurements(j,6:8)' - (est_v_eb_e_old - x(4:6,1)));   
 
    pred_vel = range_rate + x(17); 
    
    y = [pred_range;pred_vel];


