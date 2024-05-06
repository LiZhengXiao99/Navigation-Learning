function [xkk,Pkk,ukk,Ukk,D_R]=improved_VAKF(xkk,Pkk,ukk,Ukk,F,G,H,z,Q,N,rou)

% A Computationally Efficient Variational Adaptive Kalman Filter for Transfer Alignment

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xkk : state estimate
% Pkk : estimation error covariance matrix
% ukk : dof parameter of IW distribution
% Ukk : inverse scale matrix of IW distribution
% F   : state transition matrix
% G   : state noise coefficient matrix
% H   : observation matrix
% z   : measurement vector
% Q   : state noise covariance matrix
% N   : number of iterations
% rou : forgetting factor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Time update
xk1k=F*xkk;
Pk1k=F*Pkk*F'+G*Q*G';

uk1k=rou*ukk;
Uk1k=rou*Ukk;

% Initialization
D_R=Uk1k/uk1k;

% Measurement update
zk1k=H*xk1k;

tilde_Pzzk1k=(z-zk1k)*(z-zk1k)';

% update Rk
for i=1:N
    
    Pzzk1k=H*Pk1k*H'+D_R;
    
    Ck=D_R*inv(Pzzk1k);
    
    delta_Rk=Ck*(tilde_Pzzk1k-Pzzk1k)*Ck';
    
    Bk=D_R+delta_Rk;
    
    ukk=uk1k+1;
    
    Ukk=Uk1k+Bk;
    
    D_R=Ukk/ukk;
    
end

% update posterior
Pzzk1k=H*Pk1k*H'+D_R;

Pxzk1k=Pk1k*H';

Kk=Pxzk1k*inv(Pzzk1k);

xkk=xk1k+Kk*(z-zk1k);

Pkk=Pk1k-Kk*H*Pk1k;
    