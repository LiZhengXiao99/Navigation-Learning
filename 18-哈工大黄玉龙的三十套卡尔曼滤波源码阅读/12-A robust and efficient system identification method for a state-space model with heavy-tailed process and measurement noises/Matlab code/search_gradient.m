function [I_a,I_b,I_d,I_q,I_w,I_Q,I_r,I_v,I_R]=search_gradient(I_a,I_b,I_d,I_q,I_w,I_Q,I_r,I_v,I_R,zA,xcNA,PcNA,Pck_1kNA,E_chi,E_lamda,E_log_chi,E_log_lamda)

%%%%%%%%%%%%%%%%%%%%
ts=size(zA,2);
nx=size(xcNA,1); 
nz=size(zA,1); 
%%%%%%%%%%%%%%%%%%%%
g_a_1=0;
g_a_2=0;
%%%%%%%%%%%%%%%%%%%%
g_b_1=0;
g_b_2=0;
%%%%%%%%%%%%%%%%%%%%
g_d_1=0;
g_d_2=0;
%%%%%%%%%%%%%%%%%%%%
B=zeros(nx,1);   
C=zeros(nz,1);   
F=zeros(nx,nx);  
H=zeros(nz,nx);  
%%%%%%%%%%%%%%%%%%%%
for t=1:ts
    
    %%%%%%%Extract parameters
    xk_1N=xcNA(:,t);
    Pk_1N=PcNA(:,(t-1)*nx+1:t*nx);
    Pk_1kN=Pck_1kNA(:,(t-1)*nx+1:t*nx);
    xkN=xcNA(:,t+1);
    PkN=PcNA(:,t*nx+1:(t+1)*nx);
    z=zA(:,t);
    
    %%%%%%%Joint smoothing density
    ckN=[xk_1N;xkN];
    PcckN=[Pk_1N Pk_1kN;Pk_1kN' PkN];
    CkN=CR(ckN,PcckN);
    %%%%%%%
    delta_w=CkN(nx+1:2*nx,:)-ckf_ProssEq(I_a,I_b,CkN(1:nx,:),t);
    Delta_w=CkN(nx+1:2*nx,:)-ckf_ProssEq(I_a,I_b,CkN(1:nx,:),t)-repmat(I_q,1,4*nx);
    B=B+E_chi(t)*sum(delta_w,2)/(4*nx);
    F=F+E_chi(t).*Delta_w*Delta_w'/(4*nx);
    %%%%%%%
    for s=1:4*nx
        g_a_1=g_a_1+E_chi(t)*f_g_a_1(I_a,I_b,I_q,I_Q,CkN(1:nx,s),CkN(nx+1:2*nx,s),t)/(4*nx);
        g_a_2=g_a_2+E_chi(t)*f_g_a_2(I_Q,CkN(1:nx,s))/(4*nx);
        
        g_b_1=g_b_1+E_chi(t)*f_g_b_1(I_a,I_b,I_q,I_Q,CkN(1:nx,s),CkN(nx+1:2*nx,s),t)/(4*nx);
        g_b_2=g_b_2+E_chi(t)*f_g_b_2(I_Q,CkN(1:nx,s))/(4*nx);
    end
    
    %%%%%%%
    XkN=CR(xkN,PkN);
    delta_v=repmat(z,1,2*nx)-ckf_Mst(I_d,XkN);
    Delta_v=repmat(z,1,2*nx)-ckf_Mst(I_d,XkN)-repmat(I_r,1,2*nx);
    C=C+E_lamda(t)*sum(delta_v,2)/(2*nx);
    H=H+E_lamda(t).*Delta_v*Delta_v'/(2*nx);
    %%%%%%%
    for s=1:2*nx
        g_d_1=g_d_1+E_lamda(t)*f_g_d_1(I_d,I_r,I_R,XkN(s),z)/(2*nx);
        g_d_2=g_d_2+E_lamda(t)*f_g_d_2(I_R,XkN(s))/(2*nx);
    end
    
end

%%%%%%%%%%Solve local maximum using Newton method
I_a=I_a-0.5*g_a_1/g_a_2;
I_b=I_b-0.5*g_b_1/g_b_2;
I_d=I_d-0.5*g_d_1/g_d_2;

%%%%%%%%%%Solve local maximum using analytical method
I_q=B/sum(E_chi);
I_w=ts/(-ts-(sum(E_log_chi)-sum(E_chi)));
I_Q=F/ts;
%%%%%%%%%%
I_r=C/sum(E_lamda);
I_v=ts/(-ts-(sum(E_log_lamda)-sum(E_lamda)));
I_R=H/ts;


