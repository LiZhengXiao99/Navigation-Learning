clc
close all
clear all


load CKF_Output_Err.mat   
errors1=out_errors;
load PGAF_VS_Output_Err.mat
errors2=out_errors;

Plot_errors(errors1,errors2);
