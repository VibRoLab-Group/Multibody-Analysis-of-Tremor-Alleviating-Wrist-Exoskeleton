function output=numericalIK(c,p)
    t=0;
    u=zeros(10,1);
    nh=[1;zeros(3,1);1;zeros(3,1)];
    [~,~,~,~,TransDis,Quat]=numKinematics_ForearmExoskeletonModel_mex(t,c,p,u,nh);
    ArmEq(1:3,1)=TransDis(:,7);
    ExoEq(1:3,1)=TransDis(:,20);
    
    output=1000*[(ArmEq-ExoEq);[zeros(3,1),eye(3)]*quatMultiply(quatConj(Quat(:,7)),Quat(:,20))];
end