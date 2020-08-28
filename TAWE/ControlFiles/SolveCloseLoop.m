%% FindGeometricCloseLoop
freq=1000;
h=1/freq;
tspan=0:h:5;
f=1;
t=0;
c=zeros(40,1);
u=zeros(10,1);
nh=[1;zeros(3,1);1;zeros(3,1)];
cF=zeros(18,1);

% Sim.drawNow(t,c,p,u,nh);

tic
for ii=1:numel(tspan)-1
    t=ii*h;
    
    q1=c(1:8);
    q1dt=c(21:28);
    
    dqw=zeros(2,1);
    dqwdt=zeros(2,1);
    dqwddt=zeros(2,1);
    
    [~,~,~,~,~,~,MM,GFI,GFF,~,JacU,JacCons,CorCons,GFCons]=System_ForearmExoskeletonModel_mex(t,c,p,u,nh);
    MM=sum(MM,3);M1=MM(1:8,1:8);M2=MM(9:end,9:end);
    JacU=sum(JacU,3).';Ju1=JacU(1:8,1:8);Ju2=JacU(9:10,9:end);Juw=Ju1(1:2,1:2);
    JacCons=[zeros(12,6),eye(12)]*sum(JacCons,3);CorCons=[zeros(12,6),eye(12)]*sum(CorCons,3);
    H=sum([GFF,GFI],2);H1=H(1:8);H2=H(9:end);
    Jlam1=JacCons(:,1:8);Jlam2=JacCons(:,9:end);JlamMap=-Jlam2\Jlam1;
    Jeu=Js*JlamMap.'*Ju2.';
    Mu=M1+JlamMap.'*M2*JlamMap;
    Ms=Mu(1:2,1:2);Mc=Mu(1:2,3:end);Mr=Mu(3:end,3:end);
    Je=[eye(2),-Mc/Mr]*JlamMap.'*Ju2.';%May test Je-Jeu Here
    uPDterm=Kw*(dqw-q1(1:2))+Bw*(dqwdt-q1dt(1:2));
    um=(Juw.')\(Ms*dqwddt+Ms*uPDterm...
        - Js*(H1+JlamMap.'*H2)...
        - Js*(JlamMap.'*M2*(Jlam2\CorCons)));
    u(1:2)=um;
    
    [c,nh,cF]=rk4Hybrid(@Flow_ForearmExoskeletonModel_mex,h,f,t,c,p,u,nh,cF);
    if(rem(ii,30)==0)
%         Sim.drawNow(t,c,p,u,nh);
        numericalIK(c,p).'
    end
end
toc

p(40:45)=p(40:45)+c(9:14);
save('./ControlFiles/CloseLoopParam','p')