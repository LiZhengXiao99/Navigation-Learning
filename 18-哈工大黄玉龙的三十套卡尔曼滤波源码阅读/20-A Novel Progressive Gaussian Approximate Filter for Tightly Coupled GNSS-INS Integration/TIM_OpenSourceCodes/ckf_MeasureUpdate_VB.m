function [Zi_all_sta,real_z_all_sta]=ckf_MeasureUpdate_VB(x_est_propagated,P_matrix_propagated,...
                                     GNSS_measurements,est_r_eb_e_old,...
                                     est_v_eb_e_old,no_meas)

%%%%%%%%%%
nx=size(x_est_propagated,1);                                         

Zi_all_sta=[];
real_z_all_sta=[];

[Xi]=genetate_CPt(x_est_propagated,P_matrix_propagated);

% Loop measurements
for j = 1:no_meas

    Zi=ckf_Mst(Xi,GNSS_measurements,est_r_eb_e_old,est_v_eb_e_old,j);

    Zi_all_sta=[Zi_all_sta; Zi];
    
    real_range=GNSS_measurements(j,1);

    real_vel=GNSS_measurements(j,2);

    real_z=[real_range;real_vel];
    
    real_z_all_sta=[real_z_all_sta; real_z];

end % for j   
