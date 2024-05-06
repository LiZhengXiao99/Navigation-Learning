function z=ckf_Mst(x,GNSS_measurements,est_r_eb_e_old,est_v_eb_e_old,j)

n = size(x,2); 

for i=1:n
    
    z(:,i)=MstEq(x(:,i),GNSS_measurements,est_r_eb_e_old,est_v_eb_e_old,j);
    
end
