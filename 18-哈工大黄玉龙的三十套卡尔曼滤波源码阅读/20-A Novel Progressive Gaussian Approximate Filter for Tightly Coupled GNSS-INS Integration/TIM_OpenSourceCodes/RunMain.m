clc
clear all
close all

%% Seeding of the random number generator for reproducability.
RandStream.setGlobalStream(RandStream('mt19937ar','seed',1));
eFilter = eFilters.CKF;
INS_GNSS_Demo_10;

%% Seeding of the random number generator for reproducability.
RandStream.setGlobalStream(RandStream('mt19937ar','seed',1));
eFilter = eFilters.PGAF_VS;
INS_GNSS_Demo_10;

