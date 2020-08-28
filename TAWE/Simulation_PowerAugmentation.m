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
p(end)=0;
SolveCloseLoop; % Use this when system parameters are changed
load CloseLoopParam.mat;
KinematicRegressionSingle;

%%
HandCOMTraj.clearTimedPlot();

freq=2000;
h=1/freq;
fCtrl=(freq/200);
tspan=0:h:30;

dqwData2=zeros(2,numel(tspan),3);
uData2=zeros(2,numel(tspan),3);
posData2=zeros(8,numel(tspan),3);
velData2=zeros(8,numel(tspan),3);

gifData_c=zeros(40,numel(tspan));
gifData_nh=zeros(8,numel(tspan));

for jj=2

    f=1;
    t=0;
    c=zeros(40,1);
    u=zeros(10,1);
    nh=[1;zeros(3,1);1;zeros(3,1)];
    cF=zeros(18,1);

    Sim.drawNow(t,c,p,u,nh);
    
    dqwint=zeros(2,1);
    dqwt0=zeros(2,1);
    dqw=[0;0]/180*pi;
    dqwdt=dqw*0;
    dqwddt=dqw*0;
    
    y1int=zeros(2,1);
    y1=zeros(2,1);
    y1t0=zeros(2,1);
    y1dt=zeros(2,1);
    
    gAdptParam=zeros(3,1);
    gAdptStepPrev=zeros(3,1);
    invGamma=eye(3);
    
    tic
    for ii=1:numel(tspan)
        t=(ii-1)*h;
        
        q1=c(1:8);
        q1dt=c(21:28);
    
        dqwt0=dqw;
        
        dqw=0.5*[pi/8;pi/4].*[sin(t);cos(t)]...
            +0.25*[pi/8;pi/4].*[cos(1.5*t);cos(1.5*t)]...
            +0.25*[pi/8;pi/4].*[cos(2*t);sin(2*t)];
        
        dqwdt=0.5*[pi/8;pi/4].*[cos(t);-sin(t)]...
            +0.25*[pi/8;pi/4].*[1.5;1.5].*[-sin(1.5*t);-sin(1.5*t)]...
            +0.25*[pi/8;pi/4].*[2;2].*[-sin(2*t);cos(2*t)];
        
        dqwddt=0.5*[pi/8;pi/4].*[-sin(t);-cos(t)]...
            +0.25*[pi/8;pi/4].*[1.5;1.5].*[1.5;1.5].*[-cos(1.5*t);-cos(1.5*t)]...
            +0.25*[pi/8;pi/4].*[2;2].*[2;2].*[-cos(2*t);-sin(2*t)];
        
        dqw=1.25*dqw;
        dqwdt=1.25*dqwdt;
        dqwddt=1.25*dqwddt;
        
        [~,frameVel,~,frameJac,TransDis,Quat]=numKinematics_ForearmExoskeletonModel_mex(t,c,p,u,nh);
        
        pureRotQuat=quatMultiply(quatConj(Quat(:,2)),Quat(:,7));
        curYPR=quat2YPR(pureRotQuat);
        curYPRDot=ypr2AngVelJac(curYPR)\(quat2Mat(quatConj(Quat(:,7)))*(frameVel(4:6,7)-frameVel(4:6,2)));
        
        if(rem(ii-1,(fCtrl))==0)
            y1t0=y1;
            
            y1=curYPR([3;1]);
            y1dt=curYPRDot([3;1]);
            
            y1int=y1int+1.5*y1*fCtrl*h-0.5*y1t0*fCtrl*h;
            dqwint=dqwint+1.5*dqw*fCtrl*h-0.5*dqwt0*fCtrl*h;
            
            dError=(y1dt-dqwdt);
            pdError=-(Kw*(y1-dqw)+Bw*(y1dt-dqwdt));
            pidError=-(Iw*(y1int-dqwint))+pdError;
            
            [~,~,~,~,~,~,~,~,~,~,JacU,JacConsExo]=System_ForearmExoskeletonModel_mex(t,c,p,u,nh);
            JacU=sum(JacU,3).';Ju1=JacU(1:8,1:8);Ju2=JacU(9:10,9:end);Juw=Ju1(1:2,1:2);
            JacConsExo=[zeros(12,6),eye(12)]*sum(JacConsExo,3);
            Jlam1=JacConsExo(:,1:8);Jlam2=JacConsExo(:,9:end);
            
            JlamMap=-Jlam2\Jlam1;
            JeuError0=Js*JlamMap.'*Ju2.';
            
            velJac=yprIDExprJacFunc(p1End,curYPR,beta1);
            Jlam1(7:12,1:2)=[transIDExprJacFunc(p2End,curYPR,beta2);yprQuatConsJac(pureRotQuat,curYPR)]*[0 1; -(velJac(1,[3 1]))/velJac(2);1 0];
            JlamMap=-Jlam2\Jlam1;
            JeuError=Js*JlamMap.'*Ju2.';
            
            
            if jj==1
                if min(abs((eig(JeuError))))/max(abs((eig(JeuError)))) < 0.05
                    asdfadff
                end
                u2=JeuError\pidError;
                u(9:10)=u2;
            end
            
            if jj==2
                if min(abs((eig(JeuError))))/max(abs((eig(JeuError)))) < 0.05
                    asdfadff
                end
                gAdptJac=-(quat2Mat(Quat(:,7))*ypr2AngVelJac(curYPR)*[0 1; -(velJac(1,[3 1]))/velJac(2);1 0]).'*skew3([0;0;-1])*quat2Mat(Quat(:,7));
                gAdptForce=gAdptJac*gAdptParam;
                gAdptStep=invGamma*gAdptJac.'*(0.5*(y1-dqw)+(y1dt-dqwdt));
                gAdptParam=gAdptParam+0.5*(-gAdptStepPrev+3*gAdptStep)*fCtrl*h;
                gAdptStepPrev=gAdptStep;
               
                u2=JeuError\(pdError-gAdptForce);
                u(9:10)=u2;
            end
            
            if jj==3
                u2=JeuError0\pidError;
                u(9:10)=u2;
            end
                
        end
        

        dqwData2(:,ii,jj)=dqw;
        posData2(:,ii,jj)=[curYPR([3;1]);c(9:14)];
        velData2(:,ii,jj)=[curYPRDot([3;1]);c(29:34)];
        uData2(:,ii,jj)=u2;
        
        if jj==2
            gifData_c(:,ii)=c;
            gifData_nh(:,ii)=nh;
        end
        
        [c,nh,cF]=rk4Hybrid(@Flow_ForearmExoskeletonModel_mex,h,f,t,c,p,u,nh,cF);
        if(rem(ii,freq/30)==0)
%             Sim.drawNow(t,c,p,u,nh);
        end
    end
    toc
end

% save('./Data/augData','dqwData2','posData2','velData2','uData2')

%% Gif Make
HandCOMTraj.clearTimedPlot();
Sim.setAxesProp('ArmIMU1Frame',1*[-0.2 -0.4 -0.3;0.2 0.4 0.3],[210 15]).setPresetTF('WORLD');
Sim.drawNow(t,c,p,u,nh);
Sim.FigHandle.set('outerposition',[0 0 1000 1000])
ff = getframe;
[im,map] = rgb2ind(ff.cdata,256,'nodither');
im(:,:,1,1) = 0;
kk=0;
txtHandle=text(0,-0.2,0.075,strcat('t=',num2str(t),'s'),'fontsize',16);

for ii=1+freq*0:numel(tspan)-freq*20
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

im=im(160:end-120,160:end-160,:,:);
imwrite(im,map,'Augmentation.gif','DelayTime',0,'LoopCount',inf) %g443800

%%
load augData.mat;

figure(11)
set(gcf, 'Units', 'pixels', 'OuterPosition', [0, 0, 576*2.25, 514*1.6]);

subplot(2,2,1)
set(gca,'Position',subplotPosSet(2,2,1,0.25,0.125,0.07,0.05));
p1=plot(tspan,dqwData2(2,:,3),'--k','LineWidth',3);
hold on
p2=plot(tspan,posData2(2,:,3),'-.c','Color',[0.1,0.8,0.1],'LineWidth',4);
p3=plot(tspan,dqwData2(1,:,3),'-b','LineWidth',3);
p4=plot(tspan,posData2(1,:,3),':r','LineWidth',5);
text(28,0.8,'(a)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',24)
xlabel('Time (s)','interpreter','latex')
ylabel('$$\theta_a$$ (rad)','interpreter','latex');
ylim([-1.5 1])
legend([p3,p4,p1,p2],{'$$\theta_{r,a,1}$$','$$\theta_{a,1}$$','$$\theta_{r,a,2}$$','$$\theta_{a,2}$$'},'interpreter','latex','location','south','orientation','horizontal')
hold off
set(gca,'fontname','times new roman','FontSize',24)

subplot(2,2,2)
set(gca,'Position',subplotPosSet(2,2,2,0.25,0.125,0.07,0.05));
p1=plot(tspan,dqwData2(2,:,1),'--k','LineWidth',3);
hold on
p2=plot(tspan,posData2(2,:,1),'-.c','Color',[0.1,0.8,0.1],'LineWidth',4);
p3=plot(tspan,dqwData2(1,:,1),'-b','LineWidth',3);
p4=plot(tspan,posData2(1,:,1),':r','LineWidth',5);
text(28,0.8,'(b)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',24)
xlabel('Time (s)','interpreter','latex')
ylabel('$$\theta_a$$ (rad)','interpreter','latex');
ylim([-1.5 1])
legend([p3,p4,p1,p2],{'$$\theta_{r,a,1}$$','$$\theta_{a,1}$$','$$\theta_{r,a,2}$$','$$\theta_{a,2}$$'},'interpreter','latex','location','south','orientation','horizontal')
hold off
set(gca,'fontname','times new roman','FontSize',24)

subplot(2,2,3)
set(gca,'Position',subplotPosSet(2,2,3,0.25,0.125,0.07,0.05));
p1=plot(tspan,dqwData2(2,:,2),'--k','LineWidth',3);
hold on
p2=plot(tspan,posData2(2,:,2),'-.c','Color',[0.1,0.8,0.1],'LineWidth',4);
p3=plot(tspan,dqwData2(1,:,2),'-b','LineWidth',3);
p4=plot(tspan,posData2(1,:,2),':r','LineWidth',5);
text(28,0.8,'(c)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',24)
xlabel('Time (s)','interpreter','latex')
ylabel('$$\theta_a$$ (rad)','interpreter','latex');
ylim([-1.5 1])
legend([p3,p4,p1,p2],{'$$\theta_{r,a,1}$$','$$\theta_{a,1}$$','$$\theta_{r,a,2}$$','$$\theta_{a,2}$$'},'interpreter','latex','location','south','orientation','horizontal')
hold off
set(gca,'fontname','times new roman','FontSize',24)

subplot(2,2,4)
set(gca,'Position',subplotPosSet(2,2,4,0.25,0.125,0.07,0.05));
errorMat=dqwData2-posData2(1:2,:,:);
errorMatNorm=sum(errorMat.*errorMat,1).^0.5;
plot(tspan(1:end),errorMatNorm(:,(1:end),3),'r','LineWidth',3);
hold on
plot(tspan(1:end),errorMatNorm(:,(1:end),1),':b','LineWidth',5);
plot(tspan(1:end),errorMatNorm(:,(1:end),2),'-.b','Color',[0.1,0.8,0.1],'LineWidth',4);
text(28,0.7,'(d)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',24)
xlabel('Time (s)','interpreter','latex')
ylabel('$$||\epsilon_p||$$ (rad)','interpreter','latex');
legend({'PID with $$J_{\epsilon,u}$$','PID with $$\hat{J}_{\epsilon,u}$$','MRAC with $$\hat{J}_{\epsilon,u}$$'},'interpreter','latex','location','north','orientation','vertical')
hold off
set(gca,'fontname','times new roman','FontSize',24)

saveas(gcf,'./Figures/fig_AugCtrlResult','epsc')

