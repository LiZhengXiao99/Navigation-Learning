function [meas_f_ib_b,meas_omega_ib_b,quant_residuals] = IMU_model(tor_i,...
        true_f_ib_b,true_omega_ib_b,IMU_errors,old_quant_residuals)
%IMU_model - Simulates an inertial measurement unit (IMU body axes used
%throughout this function)
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 3/4/2012 by Paul Groves
%
% Inputs:
%   tor_i            time interval between epochs (s)
%   true_f_ib_b      true specific force of body frame w.r.t. ECEF frame, resolved
%                    along body-frame axes, averaged over time interval (m/s^2)
%   true_omega_ib_b  true angular rate of body frame w.r.t. ECEF frame, resolved
%                    about body-frame axes, averaged over time interval (rad/s)
%   IMU_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .b_a              Accelerometer biases (m/s^2)
%     .b_g              Gyro biases (rad/s)
%     .M_a              Accelerometer scale factor and cross coupling errors
%     .M_g              Gyro scale factor and cross coupling errors            
%     .G_g              Gyro g-dependent biases (rad-sec/m)             
%     .accel_noise_root_PSD   Accelerometer noise root PSD (m s^-1.5)
%     .gyro_noise_root_PSD    Gyro noise root PSD (rad s^-0.5)
%     .accel_quant_level      Accelerometer quantization level (m/s^2)
%     .gyro_quant_level       Gyro quantization level (rad/s)
%   old_quant_residuals  residuals of previous output quantization process
%
% Outputs:
%   meas_f_ib_b      output specific force of body frame w.r.t. ECEF frame, resolved
%                    along body-frame axes, averaged over time interval (m/s^2)
%   meas_omega_ib_b  output angular rate of body frame w.r.t. ECEF frame, resolved
%                    about body-frame axes, averaged over time interval (rad/s)
%   quant_residuals  residuals of output quantization process

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Generate noise
if tor_i>0
    accel_noise = randn(3,1) * IMU_errors.accel_noise_root_PSD / sqrt(tor_i);  
    gyro_noise = randn(3,1) * IMU_errors.gyro_noise_root_PSD / sqrt(tor_i);  
else
    accel_noise = [0;0;0];
    gyro_noise = [0;0;0];
end %if tor_i

% Calculate accelerometer and gyro outputs using (4.16) and (4.17)
uq_f_ib_b = IMU_errors.b_a + (eye(3) + IMU_errors.M_a) * true_f_ib_b +...
    accel_noise;
uq_omega_ib_b = IMU_errors.b_g + (eye(3) + IMU_errors.M_g) *...
    true_omega_ib_b + IMU_errors.G_g * true_f_ib_b + gyro_noise;
    
% Quantize accelerometer outputs
if IMU_errors.accel_quant_level>0
    meas_f_ib_b = IMU_errors.accel_quant_level * round((uq_f_ib_b +...
        old_quant_residuals(1:3)) / IMU_errors.accel_quant_level);
    quant_residuals(1:3,1) = uq_f_ib_b + old_quant_residuals(1:3) -...
        meas_f_ib_b;
else
    meas_f_ib_b = uq_f_ib_b;
    quant_residuals(1:3,1) = [0;0;0];
end %if IMU_errors.accel_quant_level
   
% Quantize gyro outputs
if IMU_errors.gyro_quant_level>0
    meas_omega_ib_b = IMU_errors.gyro_quant_level * round((uq_omega_ib_b +...
        old_quant_residuals(4:6)) / IMU_errors.gyro_quant_level);
    quant_residuals(4:6,1) = uq_omega_ib_b + old_quant_residuals(4:6) -...
        meas_omega_ib_b;
else
    meas_omega_ib_b = uq_omega_ib_b;
    quant_residuals(4:6,1) = [0;0;0];
end %if IMU_errors.gyro_quant_level

% Ends