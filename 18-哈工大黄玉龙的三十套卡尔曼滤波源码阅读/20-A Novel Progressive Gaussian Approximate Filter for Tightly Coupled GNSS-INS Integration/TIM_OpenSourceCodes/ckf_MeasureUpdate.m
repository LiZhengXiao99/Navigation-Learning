function [x_est_new,P_matrix_new]=ckf_MeasureUpdate(x_est_propagated,P_matrix_propagated,...
                                     GNSS_measurements,est_r_eb_e_old,...
                                     est_v_eb_e_old,no_meas,R)

%%%%%%%%%%
nx=size(x_est_propagated,1);                                         

nPts=2*nx;    

Zi_all_sta=[];
zkk1_all_sta=[];
real_z_all_sta=[];

[Xi]=genetate_CPt(x_est_propagated,P_matrix_propagated);

% Loop measurements
for j = 1:no_meas

    Zi=ckf_Mst(Xi,GNSS_measurements,est_r_eb_e_old,est_v_eb_e_old,j);

    zkk1=sum(Zi,2)/nPts;

    real_range=GNSS_measurements(j,1);

    real_vel=GNSS_measurements(j,2);

    real_z=[real_range;real_vel];

    Zi_all_sta=[Zi_all_sta; Zi];

    zkk1_all_sta=[zkk1_all_sta;zkk1];

    real_z_all_sta=[real_z_all_sta; real_z];

end % for j   

delta_z = real_z_all_sta - zkk1_all_sta;

Pzzkk1 = Zi_all_sta*Zi_all_sta'/nPts-zkk1_all_sta*zkk1_all_sta'+R;

Pxzkk1 = Xi*Zi_all_sta'/nPts-x_est_propagated*zkk1_all_sta';

K_matrix = Pxzkk1*inv(Pzzkk1);            

x_est_new = x_est_propagated + K_matrix * delta_z;

P_matrix_new = P_matrix_propagated-K_matrix*Pzzkk1*K_matrix'; 