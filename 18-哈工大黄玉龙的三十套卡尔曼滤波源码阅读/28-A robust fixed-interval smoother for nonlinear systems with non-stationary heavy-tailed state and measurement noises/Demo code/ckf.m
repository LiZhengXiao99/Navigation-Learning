function [xkk,Skk,xkk1,Pkk1,Pkk1k]=ckf(xkk,Skk,F,z,Q,R)

%%%%%%%%%
nx=size(xkk,1);    

nPts=2*nx;    

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%%%%%%%Time update
xk1k1=xkk;

Skk1=utchol(Skk);

 %repmat(xk1k1,1,nPts):将向量xk1k1增广到1*nPts块
Xk1k1=repmat(xk1k1,1,nPts)+Skk1*CPtArray;

Xkk1=ckf_ProssEq(Xk1k1);                                

xkk1=sum(Xkk1,2)/nPts;  %对行求和      
          
Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;   

Pkk1k=(Xk1k1-repmat(xk1k1,1,nPts))*(Xkk1-ckf_ProssEq(repmat(xk1k1,1,nPts)))'/nPts;
% (repmat(z,1,2*nx)-ckf_Mst(XkN))

% %%%%%%%%%Time update
% xk1k1=xkk;
% 
% Sk1k1=Skk;
% 
% xkk1=F*xk1k1;
% 
% Pkk1=F*Sk1k1*F'+Q;
% 
% Pkk1=(Pkk1+Pkk1')/2;        %%%%%Keep symmetry
% 
% Pkk1k=Sk1k1*F';

%%%%%%%%%Measurement update
Xi=CR(xkk1,Pkk1);

Zi=ckf_Mst(Xi);
    
zkk1=sum(Zi,2)/nPts;           

Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R;

Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);            
  
xkk=xkk1+Wk*(z-zkk1);  

Skk=Pkk1-Wk*Pzzkk1*Wk';   


   