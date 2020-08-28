
mode=p(end);
pReal=p(28:33);

beta1=[10 1e0 1e0 1e0 0 1].';
beta2=[0 1e0 0 0 0 0].';
    
yprTrajGen;

t=0;
c=zeros(40,1);
u=zeros(10,1);
nh=[1;zeros(3,1);1;zeros(3,1)];
cF=zeros(18,1);

cList=[wristTraj;zeros(18,numel(tSpan));wristTrajDot;zeros(18,numel(tSpan))];

noiseAmp=2;

yprData0=zeros(3,numel(tSpan));
transData0=zeros(3,numel(tSpan));
yprData=zeros(6,numel(tSpan));
transData=zeros(6,numel(tSpan));
for ii=1:numel(tSpan)
    [~,~,~,frameJac,TransDis,Quat]=numKinematics_ForearmExoskeletonModel_mex(t,cList(:,ii),p,u,nh);
    transData0(1:3,ii)=quat2Mat(quatConj(Quat(:,2)))*(TransDis(:,7)-TransDis(:,2));
    transData(1:3,ii)=quat2Mat(quatConj(Quat(:,2)))*(TransDis(:,7)-TransDis(:,2))+(2*rand(3,1)-1)*0.25e-2*noiseAmp^(2-noiseAmp);
    yprData0(1:3,ii)=quat2YPR(quatMultiply(quatConj(Quat(:,2)),Quat(:,7)));
    yprData(1:3,ii)=quat2YPR(quatMultiply(quatConj(Quat(:,2)),Quat(:,7)))+(2*rand(3,1)-1)*pi/180*noiseAmp^(2-noiseAmp);
    transData(4:6,ii)=quat2Mat(quatConj(Quat(:,2)))*(frameJac(1:3,:,7)-frameJac(1:3,:,2))*cList((end/2+1):end,ii);
    yprData(4:6,ii)= qAng2rAngJacFunc(pReal,[wristTraj(1,ii);wristTraj(2,ii)],mode)*wristTrajDot(:,ii);
%     (yprData(1:3,ii)-qAng2rAngFunc(pReal,wristTraj(:,ii))).'; %Double check if the results are the same
end

if noiseAmp==2
yprData(1,:)=lowpass(yprData(1,:),2,200);
yprData(2,:)=lowpass(yprData(2,:),2,200);
yprData(3,:)=lowpass(yprData(3,:),2,200);
transData(1,:)=lowpass(transData(1,:),2,200);
transData(2,:)=lowpass(transData(2,:),2,200);
transData(3,:)=lowpass(transData(3,:),2,200);
end

cutLength=20;
tSpanTrain=tSpan(cutLength+1:end-cutLength);
yprData0=yprData0(:,cutLength+1:end-cutLength);
transData0=transData0(:,cutLength+1:end-cutLength);
yprData=yprData(:,cutLength+1:end-cutLength);
transData=transData(:,cutLength+1:end-cutLength);

curveFitOpts = optimoptions('lsqcurvefit',...
                            'FunctionTolerance',1e-12,...
                            'StepTolerance',1e-12,...
                            'Algorithm','levenberg-marquardt',...
                            'SpecifyObjectiveGradient',true,...
                            'MaxIterations',500,...
                            'Display','iter');

regFunc1=@(p_,x_,beta_) deal(yprIDExprFunc(p_,x_,beta_),yprJacSort(p_,x_,beta_));
[p1End] = lsqcurvefit(@(p_,x_) regFunc1(p_,x_,beta1),zeros(19,1),yprData(1:3,:),0*tSpanTrain,[],[],curveFitOpts);

regFunc2=@(p_,x_,beta_) deal(transIDExprFunc(p_,x_,beta_),transJacSort(p_,x_,beta_));
[p2End] = lsqcurvefit(@(p_,x_) regFunc2(p_,x_,beta2),zeros(114,1),yprData(1:3,:),transData(1:3,:),[],[],curveFitOpts);
        
yprTrajGen;

cList=[wristTraj;zeros(18,numel(tSpan));wristTrajDot;zeros(18,numel(tSpan))];
rData=zeros(6,numel(tSpan));
rData2=zeros(1,numel(tSpan));
tData=zeros(6,numel(tSpan));
tData2=zeros(3,numel(tSpan));

for ii=1:numel(tSpan)
    [~,~,~,frameJac,TransDis,Quat]=numKinematics_ForearmExoskeletonModel_mex(t,cList(:,ii),p,u,nh);
    tData(1:3,ii)=quat2Mat(quatConj(Quat(:,2))).'*(TransDis(:,7));
    rData(1:3,ii)=quat2YPR(quatMultiply(quatConj(Quat(:,2)),Quat(:,7)));
    tData(4:6,ii)=quat2Mat(quatConj(Quat(:,2))).'*(frameJac(1:3,:,7)-frameJac(1:3,:,2))*cList((end/2+1):end,ii);
    rData(4:6,ii)=qAng2rAngJacFunc(pReal,wristTraj(:,ii),mode)*wristTrajDot(:,ii);

    velJac=yprIDExprJacFunc(p1End,rData(1:3,ii),beta1);
    rData2(ii)=-(velJac(1,[1 3])*rData([4 6],ii))/velJac(2);
    tData2=transIDExprDotFunc(p2End,rData,beta2);
end

transError=(tData(4:6,:)-tData2(1:3,:));
angError=rData(5,:)-rData2;

figure(20)
subplot(1,3,1)
plot(tSpan,rData(1:3,:))
xlabel('Time (s)')
ylabel('Euler Angles (rad)');
legend('Roll','Pitch','Yaw');
subplot(1,3,2)
plot(tSpan,[rData(5,:);rData2])
xlabel('Time (s)')
ylabel('Yaw Velocity (rad/s)');
legend('Yaw-Real','Yaw-Estimated');
subplot(1,3,3)
plot(tSpan,[rData(5,:)-rData2])
xlabel('Time (s)')
ylabel('Yaw Velocity Error (rad/s)');

figure(21)
subplot(2,3,1)
plot(tSpan,100*[tData(4,:);tData2(1,:)])
xlabel('Time (s)')
ylabel('Velocity X (cm/s)');
subplot(2,3,2)
plot(tSpan,100*[tData(5,:);tData2(2,:)])
xlabel('Time (s)')
ylabel('Velocity Y (cm/s)');
subplot(2,3,3)
plot(tSpan,100*[tData(6,:);tData2(3,:)])
xlabel('Time (s)')
ylabel('Velocity Z (cm/s)');
subplot(2,3,4)
plot(tSpan,100*transError(1,:))
xlabel('Time (s)')
ylabel('Vel. Est. Error X (cm/s)');
subplot(2,3,5)
plot(tSpan,100*transError(2,:))
xlabel('Time (s)')
ylabel('Vel. Est. Error Y (cm/s)');
subplot(2,3,6)
plot(tSpan,100*transError(3,:))
xlabel('Time (s)')
ylabel('Vel. Est. Error Z (cm/s)');
drawnow

%%
function jac_=yprJacSort(p_,x_,beta_)
    exprNum_=1;
    paramNum_=numel(p_);
    num_=numel(x_(1,:));
    jac_=zeros(exprNum_*num_,paramNum_);
    for ii_=1:num_
        jac_((exprNum_*ii_-exprNum_+1):(exprNum_*ii_),:)=yprIDParamJacFunc(p_,x_(:,ii_),beta_);
    end
end

function jac_=transJacSort(p_,x_,beta_)
    exprNum_=3;
    paramNum_=numel(p_);
    num_=numel(x_)/3;
    jac_=zeros(exprNum_*num_,paramNum_);
    for ii_=1:num_
        jac_((exprNum_*ii_-exprNum_+1):(exprNum_*ii_),:)=transIDParamJacFunc(p_,x_(:,ii_),beta_);
    end
end