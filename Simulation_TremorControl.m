PathSetup;
% close all;

syms q1 q2 real
param=sym('p%d',[19,1],'real');
syms(param);
syms r1 r2 r3 real
syms mode real
ypr=[r1;r2;r3];

matReal=ypr2Mat([p1;p2;p3])*ypr2Mat([(mode)*q2;0;(1-mode)*q1])*ypr2Mat([p4;p5;p6])*ypr2Mat([(1-mode)*q2;0;(mode)*q1]);
matRead=ypr2Mat(ypr);

syms sq1 cq1 sq2 cq2 real 

qAng2rAng(3,1)=atan2(matReal(2,1),matReal(1,1));
qAng2rAng(2,1)=atan2(-matReal(3,1),sqrt((matReal(3,2).^2+matReal(3,3).^2)));
qAng2rAng(1,1)=atan2(matReal(3,2),matReal(3,3));
qAng2rAngJac=jacobian(qAng2rAng,[q1,q2]);
qAng2rAngFunc=matlabFunction(qAng2rAng,'vars',{[p1;p2;p3;p4;p5;p6],[q1;q2],mode});
qAng2rAngJacFunc=matlabFunction(qAng2rAngJac,'vars',{[p1;p2;p3;p4;p5;p6],[q1;q2],mode});


%%
Sim=sAxes('ForearmExo Simulation',3,'Path_ForearmExoskeletonModel.mat',@numTF_ForearmExoskeletonModel);
Sim.setAxesProp('ArmIMU1Frame',1*[-0.2 -0.1 -0.2;0.2 0.5 0.2],[210 15]).setPresetTF('WORLD');

[ArmPose,ExoPose,HandCOMTraj]...
    =Sim.genPlot({'ArmPose';'ExoPose';'HandCOMTraj'});
ArmPose.setLineSpec('-','none','r',2).setPlotPoint({'ArmIMU1Frame';'ArmDevFrame';'ArmFlexFrame';'ArmIMU2Frame'});
ExoPose.setLineSpec('-','none','b',2).setPlotPoint({'ExoIMU1Frame';'ExoMedFrame';'ExoPanFrame';'ExoLink12Frame';...
                                                    'ExoLink23Frame';'ExoLink34Frame';'ExoLink45Frame';'ExoAttachFrame';'ExoIMU2Frame'});
HandCOMTraj.setLineSpec('-','none','g',1).setTimedPlotPoint('ExoLink34Frame');

[ArmBaseCAD,ArmHandCAD...
     ,ExoBaseCAD,ExoPanCAD,ExoLink1CAD,ExoLink2CAD,...
     ExoLink3CAD,ExoLink4CAD,ExoLinkJointCAD...
    ]...
=Sim.genPatch({
               'ArmBaseCAD'  'ArmIMU1Frame';...
               'ArmHandCAD'  'ArmIMU2AdjustFrame';...
               'ExoBaseLink'  'ExoIMU1Frame';...
               'ExoPan'  'ExoPanFrame';...
               'ExoLink1'  'ExoLink12Frame';...
               'ExoLink2'  'ExoLink23Frame';...
               'ExoLink3'  'ExoLink34Frame';...
               'ExoLink4'  'ExoLink45Frame';...
               'ExoLinkJoint'  'ExoAttachFrame';...
               });
ArmBaseCAD.setFaceProp([0.8 0.5 0.5],0.5).setModel('ArmBase.STL',0.1,[],0.001);
ArmHandCAD.setFaceProp([0.8 0.5 0.5],0.5).setModel('ArmHand.STL',0.1,[],0.001);
ExoBaseCAD.setFaceProp([0.5 0.5 0.8],0.5).setModel('ExoBaseLink.STL',0.1,[],1);
ExoPanCAD.setFaceProp([0.5 0.5 0.8],0.5).setModel('ExoPan.STL',0.1,[],1);
ExoLink1CAD.setFaceProp([0.5 0.5 0.8],0.5).setModel('ExoLink1.STL',0.025,[],0.001);
ExoLink2CAD.setFaceProp([0.5 0.5 0.8],0.5).setModel('ExoLink2.STL',0.025,[],0.001);
ExoLink3CAD.setFaceProp([0.5 0.5 0.8],0.5).setModel('ExoLink3.STL',0.025,[],0.001);
ExoLink4CAD.setFaceProp([0.5 0.5 0.8],0.5).setModel('ExoLink4.STL',0.025,[],0.001);
ExoLinkJointCAD.setFaceProp([0.5 0.5 0.8],0.5).setModel('ExoLinkJoint.STL',1,[],0.001);
Sim.Patch(1).setFaceProp([0.8 0.8 1],1).setModel([],1,[],0.1);

ControlParameters;
SystemParameters;
p(end)=rand(1);
SolveCloseLoop; % Use this when system parameters are changed
load CloseLoopParam.mat;
KinematicRegressionSingle;

%%
% HandCOMTraj.clearTimedPlot();

freq=2000;
h=1/freq;
fCtrl=(freq/200);
tspan=0:h:20;

dqwData2=zeros(2,numel(tspan),3);
uData2=zeros(2,numel(tspan),3);
posData2=zeros(8,numel(tspan),3);
velData2=zeros(8,numel(tspan),3);

gifData_c=zeros(40,numel(tspan));
gifData_nh=zeros(8,numel(tspan));

% NoiseGenerations; % Use this when data requires changes
load('./ControlFiles/GenNoiseData.mat');

for jj=3
    f=2;
    t=0;
    c=zeros(40,1);
    u=zeros(10,1);
    nh=[1;zeros(3,1);1;zeros(3,1)];
    cF=zeros(18,1);

%     Sim.drawNow(t,c,p,u,nh);
    
    dqwint=zeros(2,1);
    dqwt0=zeros(2,1);
    dqw=[0;0]/180*pi;
    dqwdt=dqw*0;
    dqwddt=dqw*0;
    
    adptParam=zeros(68,1);
    adptStepPrev=zeros(68,1);
    adptGain=0.2*eye(68);
    
    u1=zeros(8,1);
    u2=zeros(2,1);
    
    tic
    for ii=1:numel(tspan)
        t=ii*h;
        q1=c(1:8);
        q1dt=c(21:28);
        
        wristNoiseTorque=0.2*[noiseData(1).data(ii);noiseData(2).data(ii)];
        
        [~,frameVel,~,frameJac,TransDis,Quat]=numKinematics_ForearmExoskeletonModel_mex(t,c,p,u,nh);
        pureRotQuat=quatMultiply(quatConj(Quat(:,2)),Quat(:,7));
        curYPR=quat2YPR(pureRotQuat);
        curYPRDot=ypr2AngVelJac(curYPR)\(quat2Mat(quatConj(Quat(:,7)))*(frameVel(4:6,7)-frameVel(4:6,2)));

        if(rem(ii-1,fCtrl)==0)
            y1=curYPR([3;1]);
            y1dt=curYPRDot([3;1]);
            pdError=-0.1*(Kw*(y1-dqw)+Bw*(y1dt-dqwdt));
            
            uPterm=(dqw-q1(1:2));
            uDterm=(dqwdt-q1dt(1:2));

            [~,Vel,~,~,~,TransDis,MM,GFI,GFF,~,JacU,JacCons,CorCons,GFCons]=System_ForearmExoskeletonModel_mex(t,c,p,u,nh);
            MM=sum(MM,3);M1=MM(1:8,1:8);M2=MM(9:end,9:end);
            JacU=sum(JacU,3).';Ju1=JacU(1:8,1:8);Ju2=JacU(9:10,9:end);Juw=Ju1(1:2,1:2);
            JacCons=sum(JacCons,3);JacConsBase=JacCons(1:6,:);JacConsExo=JacCons(7:end,:);
            CorConsBase=CorCons(1:6);CorConsExo=CorCons(7:end);
            
            Jh=[eye(2),zeros(2,6);JacConsBase(:,1:8)];
            H=sum([GFF,GFI],2);H1=H(1:8);H2=H(9:end);
            Jlam1=JacConsExo(:,1:8);Jlam2=JacConsExo(:,9:end);
            JlamMap=-Jlam2\Jlam1;
            Mu=M1+JlamMap.'*M2*JlamMap;
            Ms=Mu(1:2,1:2);Mc=Mu(1:2,3:end);Mr=Mu(3:end,3:end);
            Me=Ms-Mc*(Mr\Mc.');
            rddtSum=-[-25*(uPterm+uDterm);CorConsBase+25/p(2)*GFCons(1:6)];
            u1=(Ju1.')\(Mu*(Jh\(rddtSum)) + ... -25/p(2)*(Jh(3:8,:).'*((Jh(3:8,:)*(Mu\Jh(3:8,:).'))\GFCons(1:6)))...
                - (H1+JlamMap.'*H2)...
                 - (JlamMap.'*M2*(Jlam2\CorConsExo)));
            
            velJac=yprIDExprJacFunc(p1End,curYPR,beta1);
            Jlam1(7:12,1:2)=[transIDExprJacFunc(p2End,curYPR,beta2);yprQuatConsJac(pureRotQuat,curYPR)]*[0 1; -(velJac(1,[3 1]))/velJac(2);1 0];
            JlamMap=-Jlam2\Jlam1;
            JeuError=Js*JlamMap.'*Ju2.';
            
            if (t>5)
                if jj==2
                    u2=JeuError\pdError;
                    u(9:10)=u2;
                end

                if jj==3

                adptJacBlock=[sin(2*pi*(4:0.25:8)*t),cos(2*pi*(4:0.25:8)*t)];
                adptJac=[adptJacBlock zeros(1,34);zeros(1,34) adptJacBlock];
                adptStep=-adptGain*adptJac.'*(0.5*(y1-dqw)+(y1dt-dqwdt));
                adptParam=adptParam+0.5*(-adptStepPrev+3*adptStep)*fCtrl*h;
                adptStepPrev=adptStep;

                u2=JeuError\pdError;
                u(9:10)=u2+JeuError\adptJac*adptParam;
                end
            end
        end
% 
        u(1:8)=u1+[wristNoiseTorque;zeros(6,1)];
        
        dqwData2(:,ii,jj)=dqw;
        posData2(:,ii,jj)=[curYPR([3;1]);c(9:14)];
        velData2(:,ii,jj)=[curYPRDot([3;1]);c(29:34)];
        uData2(:,ii,jj)=u(9:10);
        
        if jj==3
            gifData_c(:,ii)=c;
            gifData_nh(:,ii)=nh;
        end
        
        [c,nh,cF]=rk4Hybrid(@Flow_ForearmExoskeletonModel_mex,h,f,t,c,p,u,nh,cF);
        if(rem(ii,freq/25)==0)
%             Sim.drawNow(t,c,p,u,nh);
        end
    end
    toc
end

% save('./Data/supData','dqwData2','posData2','velData2','uData2')

%% Gif Make
HandCOMTraj.clearTimedPlot();
Sim.setAxesProp('ArmIMU1Frame',1*[-0.2 -0.4 -0.2;0.2 0.4 0.2],[210 15]).setPresetTF('WORLD');
Sim.drawNow(t,c,p,u,nh);
Sim.FigHandle.set('outerposition',[0 0 1000 1000])
ff = getframe;
[im,map] = rgb2ind(ff.cdata,256,'nodither');
im(:,:,1,1) = 0;
kk=0;
txtHandle=text(0,-0.2,0.075,strcat('t=',num2str(t),'s'),'fontsize',16);

for ii=1+freq*0:numel(tspan)-freq*10
    if(rem(ii,freq/25)==1)
        t=ii*h-h;
        c=gifData_c(:,ii);
        nh=gifData_nh(:,ii);
        Sim.drawNow(t,c,p,u,nh);
        delete(txtHandle)
        txtHandle=text(0+c(3),-0.2+c(4),0.075+c(5),strcat('t=',num2str(t),'s'),'fontsize',16);
        
        ff = getframe;
        kk=kk+1;
        im(1:size(ff.cdata)*[1;0;0],1:size(ff.cdata)*[0;1;0],1,kk) = rgb2ind(ff.cdata,map,'nodither');
    end
end
delete(txtHandle)

im=im(180:end-180,200:end-180,:,:);
imwrite(im,map,'Tremor.gif','DelayTime',0,'LoopCount',inf) %g443800

%% 
load supData.mat;

figure(1)
set(gcf, 'Units', 'pixels', 'OuterPosition', [0, 0, 576*2.25, 514*1.6]);

subplot(2,1,1)
set(gca,'Position',subplotPosSet(2,1,1,0.25,0.075,0.07,0.05));
errorMat=dqwData2-posData2(1:2,:,:);
errorMatNorm=sum(errorMat.*errorMat,1).^0.5;
plot(tspan(1:end),errorMatNorm(:,(1:end),1),'--k','LineWidth',1);
hold on
plot(tspan(1:end),errorMatNorm(:,(1:end),2),':r','LineWidth',3);
plot(tspan(1:end),errorMatNorm(:,(1:end),3),'b','LineWidth',3);
text(1,0.5*0.9,'(a)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',24)
xlabel('Time (s)','interpreter','latex')
ylabel('$$||\epsilon_p||$$ (rad)','interpreter','latex');
legend({'No Control','Passive (PD)','Active (MRAC)'},'interpreter','latex','location','north','orientation','horizontal')
ylim([0 0.5])
hold off
set(gca,'fontname','times new roman','FontSize',24)

subplot(2,1,2)
inputMatNorm=sum(uData2.*uData2,1).^0.5;
set(gca,'Position',subplotPosSet(2,1,2,0.25,0.075,0.07,0.05));
plot(tspan(1:end),inputMatNorm(:,(1:end),1),'--k','LineWidth',1);
hold on
plot(tspan(1:end),inputMatNorm(:,(1:end),3),':r','LineWidth',3);
plot(tspan(1:end),inputMatNorm(:,(1:end),2),'b','LineWidth',3);
text(1,1.08,'(b)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',24)
xlabel('Time (s)','interpreter','latex')
ylabel('$$||u_e||$$ (Nm)','interpreter','latex');
legend({'No Control','Passive (PD)','Active (MRAC)'},'interpreter','latex','location','north','orientation','horizontal')
ylim([0 1.2])
hold off
set(gca,'fontname','times new roman','FontSize',24)

saveas(gcf,'./Figures/fig_supCtrlResult','epsc')

