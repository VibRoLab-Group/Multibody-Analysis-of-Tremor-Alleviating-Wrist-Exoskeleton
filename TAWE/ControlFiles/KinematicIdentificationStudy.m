% load ModelInfo_ForearmExoskeletonModel.mat
% t=ModelInfo.Variables.Time;
% q=ModelInfo.Variables.Continuous(1:end/2);
% qdot=ModelInfo.Variables.Continuous(end/2+1:end);
% 
% ConJac=ModelInfo.Constraints.ConsJacobian;
% yDis=ModelInfo.CustomInfo.ArmAttachEnd;
% yVel=ConJac(end-2:end,1:8)*qdot(1:8);
% 
% transKnown=[yDis;yVel];
% knownParam=[eye(9),zeros(9,9);zeros(6,12),eye(6);[zeros(1,9),1,zeros(1,8)]]*symvar(yDis).';
% unknownParam=[eye(9),zeros(9,9);zeros(6,12),eye(6)]*symvar(yDis).';
% transUnknown=subs(transKnown,[zeros(1,9),1,zeros(1,8)]*symvar(yDis).',0);
% jacUnknown=jacobian(transUnknown,unknownParam);
% 
% matlabFunction(transKnown,'file','./ControlFiles/transKnownFunc.m','vars',{knownParam,ModelInfo.Variables.Continuous}); 
% matlabFunction(transUnknown,'file','./ControlFiles/transUnknownFunc.m','vars',{unknownParam,ModelInfo.Variables.Continuous}); 
% matlabFunction(jacUnknown,'file','./ControlFiles/jacUnknownFunc.m','vars',{unknownParam,ModelInfo.Variables.Continuous}); 

%% Offline Regression Test

% load CloseLoopParam.mat
% load idData.mat
% alpha=1;
% alphaList=(alpha.^linspace(2499,0,2500)).';
% alphaListMultiplier=reshape([alphaList,alphaList,alphaList,alphaList,alphaList,alphaList].',[],1);
% idFunc=@(p,c) deal(alphaListMultiplier.*(reshape([eye(3),zeros(3);zeros(3),0.2*eye(3)]*transUnknownFunc(p,c),[],1)),(alphaListMultiplier.^0.5).*jacUnknownFuncSort(p,c));
% optOpts = optimoptions('lsqcurvefit',...
%                        'Algorithm','levenberg-marquardt',...
%                        'FunctionTolerance',1e-12,...
%                        'StepTolerance',1e-12,...
%                        'SpecifyObjectiveGradient',true,...
%                        'Display','off'...
%                        );
%                    
% x0=0*[0.01;0.01;-0.01;0;0;0;-0.01;0.01;0.01;0;0;0;0;0;0];
%                   
% idParam=zeros(16,numel(idData));
% errorSum=0;
% 
% for jj=1:numel(idData)
%     
% trueParam=[p(8:16);p(23:28)];
% noiseData=[eye(3),zeros(3);zeros(3),0.2*eye(3)]*transKnownFunc([trueParam;p(end)],idData(jj).cList);
% noiseData=noiseData+[eye(3),zeros(3);zeros(3),0.2*eye(3)]*0.001*[eye(3),zeros(3);zeros(3),5*eye(3)]*rand(size(noiseData));
% noiseData=reshape(noiseData,[],1);
% tic
% x = lsqcurvefit(idFunc,x0,idData(jj).cList,alphaListMultiplier.*noiseData,[],[],optOpts);
% toc
% disp([x0,x,trueParam].')
% disp([norm(x0-trueParam),norm(x-trueParam),max(max(transUnknownFunc(x,idData(jj).cList)-transKnownFunc([trueParam;p(end)],idData(jj).cList)))])
% 
% idParam(:,jj)=[x;0];
% 
% error=[eye(3),zeros(3)]*(transUnknownFunc(x,idData(jj).cList)-transKnownFunc([trueParam;p(end)],idData(jj).cList));
% errorNorm=sum(error.*error,1).^0.5;
% figure(50)
% plot(linspace(1/250,10,2500),sum(error.*error,1).^0.5,'--','linewidth',1);
% drawnow;
% 
% end
% 
% save('./ControlFiles/idParam.mat','idParam')

%% Offline Regression Test 2

load CloseLoopParam.mat
load idData.mat
alpha=1;
alphaList=(alpha.^linspace(2499,0,2500)).';
alphaListMultiplier=reshape([alphaList,alphaList,alphaList,alphaList,alphaList,alphaList].',[],1);
idFunc=@(p,c) deal(alphaListMultiplier.*(reshape([eye(3),zeros(3);zeros(3),0*0.2*eye(3)]*transUnknownFunc(p,c),[],1)),(alphaListMultiplier.^0.5).*jacUnknownFuncSort(p,c));
optOpts = optimoptions('lsqcurvefit',...
                       'Algorithm','levenberg-marquardt',...
                       'FunctionTolerance',1e-12,...
                       'StepTolerance',1e-12,...
                       'SpecifyObjectiveGradient',true,...
                       'Display','off'...
                       );
                   
x0=0*[0.01;0.01;-0.01;0;0;0;-0.01;0.01;0.01;0;0;0;0;0;0];
x0(10:12)=[0;pi/2;0];
x0(13:15)=[0;-pi;0];
                  
idParam=zeros(16,numel(idData));
errorSum=0;

for jj=1:numel(idData)
    
cListAlt=idData(jj).cList;
    
cListAlt(1:2,:)=vecRotAng([  cos(cListAlt(1,:)).*sin(cListAlt(2,:)); -sin(cListAlt(1,:)) ;cos(cListAlt(1,:)).*cos(cListAlt(2,:))]);
cListAlt(18:19,:)=0*cListAlt(18:19,:);
    
trueParam=[p(8:16);p(23:28)];
noiseData=[eye(3),zeros(3);zeros(3),0*0.2*eye(3)]*transKnownFunc([trueParam;p(end)],idData(jj).cList);
noiseData=noiseData+0*[eye(3),zeros(3);zeros(3),0*0.2*eye(3)]*0.001*[eye(3),zeros(3);zeros(3),5*eye(3)]*rand(size(noiseData));
noiseData=reshape(noiseData,[],1);
tic
x = lsqcurvefit(idFunc,x0,cListAlt,alphaListMultiplier.*noiseData,[],[],optOpts);
toc
disp([x0,x,trueParam].')
disp([norm(x0-trueParam),norm(x-trueParam),max(max(transUnknownFunc(x,cListAlt)-transKnownFunc([trueParam;p(end)],idData(jj).cList)))])

idParam(:,jj)=[x;0];

error=[eye(3),zeros(3)]*(transUnknownFunc(x,cListAlt)-transKnownFunc([trueParam;p(end)],idData(jj).cList));
errorNorm=sum(error.*error,1).^0.5;
figure(50)
plot(linspace(1/250,10,2500),sum(error.*error,1).^0.5,'--','linewidth',1);
drawnow;

end

save('./ControlFiles/idParam2.mat','idParam')


%% 
load idData.mat
load idParam.mat
load CloseLoopParam.mat

idParam=idParam(:,1:50);
paramAverage=sum(idParam,2)/50;
pd1ParamErrNorm=sqrt(sum([eye(3),zeros(3,13)]*(idParam-paramAverage).^2,1));
pd2ParamErrNorm=sqrt(sum([zeros(3),eye(3),zeros(3,10)]*(idParam-paramAverage).^2,1));
pd3ParamErrNorm=sqrt(sum([zeros(3,6),eye(3),zeros(3,7)]*(idParam-paramAverage).^2,1));
plot(1:50,[pd1ParamErrNorm;pd2ParamErrNorm;pd3ParamErrNorm])

pr1ParamErrNorm=sqrt(sum([zeros(3,9),eye(3),zeros(3,4)]*(idParam-paramAverage).^2,1));
pr2ParamErrNorm=sqrt(sum([zeros(3,12),eye(3),zeros(3,1)]*(idParam-paramAverage).^2,1));
plot(1:50,[pr1ParamErrNorm;pr2ParamErrNorm])




alpha=0.999;
alphaList=(alpha.^linspace(2499,0,2500)).';
alphaListMultiplier=reshape([alphaList,alphaList,alphaList].',[],1);

trueParam=[p(8:16);p(23:28);p(end)];
posErrorNormList=zeros(numel(idParam)/16,numel(idParam)/16);
maxNormList=zeros(numel(idParam)/16,numel(idParam)/16);
areaListComparison=zeros(numel(idParam)/16,numel(idParam)/16);

for ii=1:numel(idParam)/16
    
    x=idParam(1:15,ii);
    
    for jj=1:numel(idParam)/16

    error=[eye(3),zeros(3)]*(transUnknownFunc(x,idData(jj).cList)-transKnownFunc([trueParam;p(end)],idData(jj).cList));
    errorNorm=sum(error.*error,1);
    
    trajii=[idData(ii).cList(1:2,:)];
    vii=convhull(trajii(1,:),trajii(2,:));
    trajjj=[idData(jj).cList(1:2,:)];
    vjj=convhull(trajjj(1,:),trajjj(2,:));
    comTraj=[trajii,trajjj];
    vTotal=convhull(comTraj(1,:),comTraj(2,:));
    
    areai=polyarea(trajii(1,vii),trajii(2,vii));
    areaj=polyarea(trajjj(1,vjj),trajjj(2,vjj));
    areaTotal=polyarea(comTraj(1,vTotal),comTraj(2,vTotal));
    
    areaListComparison(ii,jj)=(areai+areaj-areaTotal)/areaTotal;
    
    posErrorNormList(ii,jj)=sum(errorNorm);
    maxNormList(ii,jj)=max(errorNorm);

    end
end

[worstFitRow,worstFitCol]= find(maxNormList==max(max(maxNormList)));
[smallestFitRow,smallestFitCol]= find(areaListComparison==min(min(areaListComparison)));
x=idParam(1:15,worstFitRow(1));
figure(1)
set(gcf, 'Units', 'pixels', 'OuterPosition', [0, 0, 576*2.5, 514*1]);
subplot(1,2,1)
% stem(1:50,sum(areaListComparison,2)/50)
% axis([0 51 0 1])
% set(gca,'fontname','times new roman','FontSize',20)
% xlabel('Sample No.','interpreter','latex')
% ylabel('$$\mathcal{A}$$','interpreter','latex')

% subplot(2,1,2)
stem(1:50,sum(posErrorNormList,2)/50)
axis([0 51 0 4e-3])
set(gca,'fontname','times new roman','FontSize',26)
xlabel('Sample No.','interpreter','latex')
ylabel('$${\mathcal{K}}$$','interpreter','latex')
text(25,3.5e-3,'(a)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',28)
% saveas(gcf,'./Figures/GeneralComparison','epsc')



% figure(2)
subplot(1,2,2)
% set(gcf, 'Units', 'pixels', 'OuterPosition', [0, 0, 576*1, 514*1]);
plot(linspace(1/250,10,2500),1000*[1,zeros(1,5)]*(transUnknownFunc(x,idData(jj).cList)-transKnownFunc([trueParam;p(end)],idData(jj).cList)),...
    '-','color','r','linewidth',2)
hold on
plot(linspace(1/250,10,2500),1000*[0,1,zeros(1,4)]*(transUnknownFunc(x,idData(jj).cList)-transKnownFunc([trueParam;p(end)],idData(jj).cList)),...
    '--','color','b','linewidth',2)
plot(linspace(1/250,10,2500),1000*[0,0,1,zeros(1,3)]*(transUnknownFunc(x,idData(jj).cList)-transKnownFunc([trueParam;p(end)],idData(jj).cList)),...
    ':','color','k','linewidth',2)
hold off
xlabel('Time (s)','interpreter','latex')
ylabel('Est. Error (mm)','interpreter','latex')
text(5,1.3125,'(b)','color','k','fontname','times new roman','fontweight','bold','horizontalalignment','center','FontSize',28)
set(gca,'fontname','times new roman','FontSize',26)
saveas(gcf,'./Figures/WorstCaseRegression','epsc')

%% Function
function output=jacUnknownFuncSort(p,c)
    cSize=size(c);
    output=zeros(3*cSize(2),15);
    for ii=1:cSize(2)
        output((6*ii-5):(6*ii),:)=[eye(3),zeros(3);zeros(3),0*0.2*eye(3)]*jacUnknownFunc(p,c(:,ii));
    end
end