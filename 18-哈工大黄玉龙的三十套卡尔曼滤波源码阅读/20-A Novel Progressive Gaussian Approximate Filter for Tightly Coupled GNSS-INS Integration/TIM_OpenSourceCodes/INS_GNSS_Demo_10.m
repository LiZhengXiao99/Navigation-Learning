%INS_GNSS_Demo_10
%SCRIPT Tightly coupled INS/GNSS demo:
%   Profile_3 (418s aircraft)
%   Low-cost consumption-grade IMU
%   Most of the code copy from "Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition", which is created 27/5/12 by Paul Groves

close all
clc

% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
micro_g_to_meters_per_second_squared = 9.80665E-6;
% CONFIGURATION
% Input truth motion profile filename
input_profile_name = 'Profile_3.csv';

% Attitude initialization error (deg, converted to rad; @N,E,D)
initialization_errors.delta_eul_nb_n = [-0.5;0.4;1]*deg_to_rad; % rad
% initialization_errors.delta_eul_nb_n = [-0.01;0.02;0.5]*deg_to_rad; % rad

% Accelerometer biases (micro-g, converted to m/s^2; body axes) 10*
IMU_errors.b_a = [9000;-13000;8000] * micro_g_to_meters_per_second_squared;
% Gyro biases (deg/hour, converted to rad/sec; body axes) 10*
IMU_errors.b_g = [-180;260;-160] * deg_to_rad / 3600;
% Accelerometer scale factor and cross coupling errors (ppm, converted to
% unitless; body axes)
IMU_errors.M_a = [50000, -15000, 10000;...
                  -7500, -60000, 12500;...
                 -12500,   5000, 20000] * 1E-6;
% Gyro scale factor and cross coupling errors (ppm, converted to unitless;
% body axes)
IMU_errors.M_g = [40000, -14000,  12500;...
                      0, -30000,  -7500;...
                      0,      0, -17500] * 1E-6;             
% Gyro g-dependent biases (deg/hour/g, converted to rad-sec/m; body axes)
IMU_errors.G_g = [90, -110,  -60;...
                 -50,  190, -160;...
                  30,  110, -130] * deg_to_rad / (3600 * 9.80665);             
% Accelerometer noise root PSD (micro-g per root Hz, converted to m s^-1.5)  10*               
IMU_errors.accel_noise_root_PSD = 1000 *...
    micro_g_to_meters_per_second_squared;
% Gyro noise root PSD (deg per root hour, converted to rad s^-0.5)   10*            
IMU_errors.gyro_noise_root_PSD = 1 * deg_to_rad / 60;
% Accelerometer quantization level (m/s^2)
IMU_errors.accel_quant_level = 1E-1;
% Gyro quantization level (rad/s)
IMU_errors.gyro_quant_level = 2E-3;

% Interval between GNSS epochs (s) 
GNSS_config.epoch_interval = 0.5;

% Initial estimated position (m; ECEF)
GNSS_config.init_est_r_ea_e = [0;0;0];

% Number of satellites in constellation
GNSS_config.no_sat = 30;
% Orbital radius of satellites (m)
GNSS_config.r_os = 2.656175E7;
% Inclination angle of satellites (deg)
GNSS_config.inclination = 55;
% Longitude offset of constellation (deg)
GNSS_config.const_delta_lambda = 0;
% Timing offset of constellation (s)
GNSS_config.const_delta_t = 0;

% Mask angle (deg)
GNSS_config.mask_angle = 10;
% Signal in space error SD (m) *Give residual where corrections are applied
GNSS_config.SIS_err_SD = 1;
% Zenith ionosphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_iono_err_SD = 2;
% Zenith troposphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_trop_err_SD = 0.2;
% Code tracking error SD (m) *Can extend to account for multipath
GNSS_config.code_track_err_SD = 1;
% Range rate tracking error SD (m/s) *Can extend to account for multipath
GNSS_config.rate_track_err_SD = 0.02;
% Receiver clock offset at time=0 (m);
GNSS_config.rx_clock_offset = 10000;
% Receiver clock drift at time=0 (m/s);
GNSS_config.rx_clock_drift = 100;

% Initial attitude uncertainty per axis (deg, converted to rad)
TC_KF_config.init_att_unc = degtorad(5);
% Initial velocity uncertainty per axis (m/s)
TC_KF_config.init_vel_unc = 0.1; % 1.2*
% Initial position uncertainty per axis (m)
TC_KF_config.init_pos_unc = 10;  % 1.2*
% Initial accelerometer bias uncertainty per instrument (micro-g, converted
% to m/s^2)
TC_KF_config.init_b_a_unc = 10000 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
TC_KF_config.init_b_g_unc = 200 * deg_to_rad / 3600;
% Initial clock offset uncertainty per axis (m)
TC_KF_config.init_clock_offset_unc = 10;
% Initial clock drift uncertainty per axis (m/s)
TC_KF_config.init_clock_drift_unc = 0.1;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s) Q                
TC_KF_config.gyro_noise_PSD = 0.01^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
TC_KF_config.accel_noise_PSD = 0.2^2;
% NOTE: A large noise PSD is modeled to account for the scale-factor and
% cross-coupling errors that are not directly included in the Kalman filter
% model
% Accelerometer bias random walk PSD (m^2 s^-5)
TC_KF_config.accel_bias_PSD = 1.0E-5;
% Gyro bias random walk PSD (rad^2 s^-3)
TC_KF_config.gyro_bias_PSD = 4.0E-11;
% Receiver clock frequency-drift PSD (m^2/s^3)
TC_KF_config.clock_freq_PSD = 1;
% Receiver clock phase-drift PSD (m^2/s)
TC_KF_config.clock_phase_PSD = 1;

% Pseudo-range measurement noise SD (m)  R 
TC_KF_config.pseudo_range_SD = 10*2.5;
% Pseudo-range rate measurement noise SD (m/s) 10*
TC_KF_config.range_rate_SD = 10*0.1;

% Begins

% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(input_profile_name);

% End script if there is a problem with the file
if ~ok
    return;
end %if

% Main Loop
nMaxRuns = 1; %MC runs---50
rmse_total = zeros(nMaxRuns, 9);

hWaitbar = waitbar(0, 'start process...');
for nRun = 1 : nMaxRuns
    % Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
    % simulation
    [out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD] =...
        Tightly_coupled_INS_GNSS(in_profile,no_epochs,...
        initialization_errors,IMU_errors,GNSS_config,TC_KF_config,eFilter);
        
    rmse_total(nRun,1) = rmse(radtodeg(out_errors(:,10)')); 
    rmse_total(nRun,2) = rmse(radtodeg(out_errors(:,9)')); 
    rmse_total(nRun,3) = rmse(radtodeg(out_errors(:,8)'));
    rmse_total(nRun,4) = rmse(out_errors(:,7)');
    rmse_total(nRun,5) = rmse(out_errors(:,6)');
    rmse_total(nRun,6) = rmse(out_errors(:,5)'); 
    rmse_total(nRun,7) = rmse(out_errors(:,4)');
    rmse_total(nRun,8) = rmse(out_errors(:,3)');
    rmse_total(nRun,9) = rmse(out_errors(:,2)');  

    waitbar(nRun/nMaxRuns);
end
close(hWaitbar);

is_rmse_mc = 0; % flag for MC runs
switch char(eFilter)
    case 'CKF' 
        if ~is_rmse_mc
            save CKF_Output_Err.mat out_errors
        else
            rmse_total_ckf = rmse_total;
            save rmse_total_CKF.mat rmse_total_ckf
        end
        disp('CKF results is saved!!!');    
case 'PGAF_VS'
    if ~is_rmse_mc
        save PGAF_VS_Output_Err.mat out_errors
    else
        rmse_total_pgafvs = rmse_total;
        save rmse_total_PGAF_VS.mat rmse_total_pgafvs
    end
    disp('PGAF_VS results is saved!!!'); 
        
end
% Ends