%% Declare Forearm Body
[ArmBaseBody,ArmWristBody,ArmHandBody]...
=System.genBody({
            'ArmBaseBody' 'ArmBaseCOM';
            'ArmWristBody' 'ArmWristCOM';
            'ArmHandBody' 'ArmHandCOM';
            });
        
IArmBaseBody=diag([P_Inert1.i11xx;P_Inert1.i11yy;P_Inert1.i11zz])...
                +[0,P_Inert1.i11xy,P_Inert1.i11xz;zeros(1,2),P_Inert1.i11yz;zeros(1,3)]...
                +[0,P_Inert1.i11xy,P_Inert1.i11xz;zeros(1,2),P_Inert1.i11yz;zeros(1,3)].';
IArmWristBody=diag([P_Inert1.i12xx;P_Inert1.i12yy;P_Inert1.i12zz])...
                +[0,P_Inert1.i12xy,P_Inert1.i12xz;zeros(1,2),P_Inert1.i12yz;zeros(1,3)]...
                +[0,P_Inert1.i12xy,P_Inert1.i12xz;zeros(1,2),P_Inert1.i12yz;zeros(1,3)].';
IArmHandBody=diag([P_Inert1.i13xx;P_Inert1.i13yy;P_Inert1.i13zz])...
                +[0,P_Inert1.i13xy,P_Inert1.i13xz;zeros(1,2),P_Inert1.i13yz;zeros(1,3)]...
                +[0,P_Inert1.i13xy,P_Inert1.i13xz;zeros(1,2),P_Inert1.i13yz;zeros(1,3)].';
            
ArmBaseBody.setProp(P_Inert1.m11,IArmBaseBody).genForce({'GArmBaseBody' ArmBaseCOM WORLD}).setProp([0;0;-P_Inert1.m11*P_Sys.g]);
ArmWristBody.setProp(P_Inert1.m12,IArmWristBody).genForce({'GArmWristBody' ArmWristCOM WORLD}).setProp([0;0;-P_Inert1.m12*P_Sys.g]);
ArmHandBody.setProp(P_Inert1.m13,IArmHandBody).genForce({'GArmHandBody' ArmHandCOM WORLD}).setProp([0;0;-P_Inert1.m13*P_Sys.g]);


%% Declare Forearm Inputs
ArmBaseBody.genTorque({'CounterArmDevTorque' ArmBaseCOM ArmDevFrame}).setProp(-[P_Others.jointOrder*U1.u1flex;0;(1-P_Others.jointOrder)*U1.u1dev]);
ArmBaseBody.genTorque({'CounterArmFlexTorque' ArmWristCOM ArmFlexFrame}).setProp(-[(1-P_Others.jointOrder)*U1.u1flex;0;P_Others.jointOrder*U1.u1dev]);
ArmHandBody.genTorque({'ArmDevTorque' ArmWristCOM ArmDevFrame}).setProp([P_Others.jointOrder*U1.u1flex;0;(1-P_Others.jointOrder)*U1.u1dev]);
ArmHandBody.genTorque({'ArmFlexTorque' ArmHandCOM ArmFlexFrame}).setProp([(1-P_Others.jointOrder)*U1.u1flex;0;P_Others.jointOrder*U1.u1dev]);

ArmBaseBody.genTorque({'ArmBaseTorque' ArmBaseCOM ArmIMU1Frame}).setProp([U1.u1rx;U1.u1ry;U1.u1rz]);
ArmBaseBody.genForce({'ArmBaseForce' ArmBaseCOM WORLD}).setProp([U1.u1dx;U1.u1dy;U1.u1dz]);


%% Declare Forearm Damper
System.genDamper({'GlobalDamper1' 6 1}).setProp(P_Sys.b,[zeros(6,2),eye(6),zeros(6,12)]*System.getContVector(1));
System.genDamper({'ArmDamper' 2 1}).setProp([P_Sys.b1dev;P_Sys.b1flex],[zeros(2,6),eye(2),zeros(2,12)]*System.getContVector(1));


%% Declare Exoskeleton Body
[ExoBaseBody,ExoPanBody,ExoLink1Body,ExoLink2Body,ExoLink3Body,ExoLink4Body,ExoLinkJointBody]...
=System.genBody({
            'ExoBaseBody' 'ExoBaseCOM';
            'ExoPanBody' 'ExoPanCOM';
            'ExoLink1Body' 'ExoLink1COM';
            'ExoLink2Body' 'ExoLink2COM';
            'ExoLink3Body' 'ExoLink3COM';
            'ExoLink4Body' 'ExoLink4COM';
            'ExoLinkJointBody' 'ExoLinkJointCOM';
            });
        
IExoBaseBody=diag([P_Inert2.i21xx;P_Inert2.i21yy;P_Inert2.i21zz])...
                +[0,P_Inert2.i21xy,P_Inert2.i21xz;zeros(1,2),P_Inert2.i21yz;zeros(1,3)]...
                +[0,P_Inert2.i21xy,P_Inert2.i21xz;zeros(1,2),P_Inert2.i21yz;zeros(1,3)].';
IExoPanBody=diag([P_Inert2.i22xx;P_Inert2.i22yy;P_Inert2.i22zz])...
                +[0,P_Inert2.i22xy,P_Inert2.i22xz;zeros(1,2),P_Inert2.i22yz;zeros(1,3)]...
                +[0,P_Inert2.i22xy,P_Inert2.i22xz;zeros(1,2),P_Inert2.i22yz;zeros(1,3)].';
IExoLink1Body=diag([P_Inert2.i23xx;P_Inert2.i23yy;P_Inert2.i23zz])...
                +[0,P_Inert2.i23xy,P_Inert2.i23xz;zeros(1,2),P_Inert2.i23yz;zeros(1,3)]...
                +[0,P_Inert2.i23xy,P_Inert2.i23xz;zeros(1,2),P_Inert2.i23yz;zeros(1,3)].';
IExoLink2Body=diag([P_Inert2.i24xx;P_Inert2.i24yy;P_Inert2.i24zz])...
                +[0,P_Inert2.i24xy,P_Inert2.i24xz;zeros(1,2),P_Inert2.i24yz;zeros(1,3)]...
                +[0,P_Inert2.i24xy,P_Inert2.i24xz;zeros(1,2),P_Inert2.i24yz;zeros(1,3)].';
IExoLink3Body=diag([P_Inert2.i25xx;P_Inert2.i25yy;P_Inert2.i25zz])...
                +[0,P_Inert2.i25xy,P_Inert2.i25xz;zeros(1,2),P_Inert2.i25yz;zeros(1,3)]...
                +[0,P_Inert2.i25xy,P_Inert2.i25xz;zeros(1,2),P_Inert2.i25yz;zeros(1,3)].';
IExoLink4Body=diag([P_Inert2.i26xx;P_Inert2.i26yy;P_Inert2.i26zz])...
                +[0,P_Inert2.i26xy,P_Inert2.i26xz;zeros(1,2),P_Inert2.i26yz;zeros(1,3)]...
                +[0,P_Inert2.i26xy,P_Inert2.i26xz;zeros(1,2),P_Inert2.i26yz;zeros(1,3)].';
IExoLinkJointBody=diag([P_Inert2.i27xx;P_Inert2.i27yy;P_Inert2.i27zz])...
                +[0,P_Inert2.i27xy,P_Inert2.i27xz;zeros(1,2),P_Inert2.i27yz;zeros(1,3)]...
                +[0,P_Inert2.i27xy,P_Inert2.i27xz;zeros(1,2),P_Inert2.i27yz;zeros(1,3)].';
            
ExoBaseBody.setProp(P_Inert2.m21,IExoBaseBody).genForce({'GExoBaseBody' ExoBaseCOM WORLD}).setProp([0;0;-P_Inert2.m21*P_Sys.g]);
ExoPanBody.setProp(P_Inert2.m22,IExoPanBody).genForce({'GExoPanBody' ExoPanCOM WORLD}).setProp([0;0;-P_Inert2.m22*P_Sys.g]);
ExoLink1Body.setProp(P_Inert2.m23,IExoLink1Body).genForce({'GExoLink1Body' ExoLink1COM WORLD}).setProp([0;0;-P_Inert2.m23*P_Sys.g]);
ExoLink2Body.setProp(P_Inert2.m24,IExoLink2Body).genForce({'GExoLink2Body' ExoLink2COM WORLD}).setProp([0;0;-P_Inert2.m24*P_Sys.g]);
ExoLink3Body.setProp(P_Inert2.m25,IExoLink3Body).genForce({'GExoLink3Body' ExoLink3COM WORLD}).setProp([0;0;-P_Inert2.m25*P_Sys.g]);
ExoLink4Body.setProp(P_Inert2.m26,IExoLink4Body).genForce({'GExoLink4Body' ExoLink4COM WORLD}).setProp([0;0;-P_Inert2.m26*P_Sys.g]);
ExoLinkJointBody.setProp(P_Inert2.m27,IExoLinkJointBody).genForce({'GExoLinkJointBody' ExoLinkJointCOM WORLD}).setProp([0;0;-P_Inert2.m27*P_Sys.g]);


%% Declare Exoskeleton Inputs
ExoBaseBody.genTorque({'ExoCounterPanTorque' ExoBaseCOM ExoPanFrame}).setProp([0;0;-U2.u21]);
ExoPanBody.genTorque({'ExoPanTorque' ExoPanCOM ExoPanFrame}).setProp([0;0;U2.u21]);
ExoPanBody.genTorque({'ExoCounterLink1Torque' ExoPanCOM ExoLink12Frame}).setProp([-U2.u22;0;0]);
ExoLink1Body.genTorque({'ExoLink1Torque' ExoLink1COM ExoLink12Frame}).setProp([U2.u22;0;0]);


%% Declare Exoskeleton Damper
System.genDamper({'GlobalDamper2' 6 1}).setProp(P_Sys.b,[zeros(6,14),eye(6)]*System.getContVector(1));
System.genDamper({'ExoActDamper' 6 1}).setProp(P_Sys.b2act,[zeros(6,8),eye(6),zeros(6,6)]*System.getContVector(1));

%% Declare Fix Armbase Mode
[LockArmBaseDx,LockArmBaseDy,LockArmBaseDz,LockArmBaseRx,LockArmBaseRy,LockArmBaseRz]...
=System.genCons({
                 'LockArmBaseDx' 'l_LABdx';
                 'LockArmBaseDy' 'l_LABdy';
                 'LockArmBaseDz' 'l_LABdz';
                 'LockArmBaseRx' 'l_LABrx';
                 'LockArmBaseRy' 'l_LABry';
                 'LockArmBaseRz' 'l_LABrz';
               });
LABPointExpr=jSimplify(ArmIMU1Frame.getTransSym(0));
LABAttitudeExpr=jSimplify(quatMultiply([1;0;0;0],quatConj(nhArm)));
LockArmBaseDx.setHCons(LABPointExpr(1),P_Sys.c);
LockArmBaseDy.setHCons(LABPointExpr(2),P_Sys.c);
LockArmBaseDz.setHCons(LABPointExpr(3),P_Sys.c);
LockArmBaseRx.setHCons(LABAttitudeExpr(2),P_Sys.c);
LockArmBaseRy.setHCons(LABAttitudeExpr(3),P_Sys.c);
LockArmBaseRz.setHCons(LABAttitudeExpr(4),P_Sys.c);

%% Declare Constraint Attach Mode
[AttachExoBaseDx,AttachExoBaseDy,AttachExoBaseDz,AttachExoBaseRx,AttachExoBaseRy,AttachExoBaseRz]...
=System.genCons({
                 'AttachExoBaseDx' 'l_AEBdx';
                 'AttachExoBaseDy' 'l_AEBdy';
                 'AttachExoBaseDz' 'l_AEBdz';
                 'AttachExoBaseRx' 'l_AEBrx';
                 'AttachExoBaseRy' 'l_AEBry';
                 'AttachExoBaseRz' 'l_AEBrz';
               });
AEBPointExpr=jSimplify(ExoIMU1Frame.getTransSym(0)-ArmIMU1Frame.getTransSym(0));
AEBAttitudeExpr=jSimplify(quatMultiply(nhExo,quatConj(nhArm)));
AttachExoBaseDx.setHCons(AEBPointExpr(1),P_Sys.c);
AttachExoBaseDy.setHCons(AEBPointExpr(2),P_Sys.c);
AttachExoBaseDz.setHCons(AEBPointExpr(3),P_Sys.c);
AttachExoBaseRx.setHCons(AEBAttitudeExpr(2),P_Sys.c);
AttachExoBaseRy.setHCons(AEBAttitudeExpr(3),P_Sys.c);
AttachExoBaseRz.setHCons(AEBAttitudeExpr(4),P_Sys.c);

[AttachExoEndDx,AttachExoEndDy,AttachExoEndDz,AttachExoEndRx,AttachExoEndRy,AttachExoEndRz]...
=System.genCons({
                 'AttachExoEndDx' 'l_AEEdx';
                 'AttachExoEndDy' 'l_AEEdy';
                 'AttachExoEndDz' 'l_AEEdz';
                 'AttachExoEndRx' 'l_AEErx';
                 'AttachExoEndRy' 'l_AEEry';
                 'AttachExoEndRz' 'l_AEErz';
               });
ArmAttachEnd=[eye(3),zeros(3,1)]*(ArmIMU2Frame.fwdTF(ArmIMU1Frame))*[zeros(3,1);1];
ExoAttachEnd=[eye(3),zeros(3,1)]*(ExoIMU2Frame.fwdTF(ExoIMU1Frame))*[zeros(3,1);1];
AEEPointExpr=ArmAttachEnd-ExoAttachEnd;
% AEEPointExpr=ExoIMU2Frame.getTransSym(0)-ArmIMU2Frame.getTransSym(0);
AttachExoEndDx.setHCons(AEEPointExpr(1),P_Sys.c);
AttachExoEndDy.setHCons(AEEPointExpr(2),P_Sys.c);
AttachExoEndDz.setHCons(AEEPointExpr(3),P_Sys.c);
% AEEAttitudeExpr(1)=[0 1 0 0]*(ArmIMU2ExtentionX.fwdTF(ArmIMU1Frame)-ExoIMU2ExtentionX.fwdTF(ExoIMU1Frame))*[zeros(3,1);1];
% AEEAttitudeExpr(2)=[0 0 1 0]*(ArmIMU2ExtentionY.fwdTF(ArmIMU1Frame)-ExoIMU2ExtentionY.fwdTF(ExoIMU1Frame))*[zeros(3,1);1];
% AEEAttitudeExpr(3)=[1 0 0 0]*(ArmIMU2ExtentionZ.fwdTF(ArmIMU1Frame)-ExoIMU2ExtentionZ.fwdTF(ExoIMU1Frame))*[zeros(3,1);1];
% AEEAttitudeExpr(1)=[0 1 0]*(ArmIMU2ExtentionX.getTransSym(0)-ExoIMU2ExtentionX.getTransSym(0));
% AEEAttitudeExpr(2)=[0 0 1]*(ArmIMU2ExtentionY.getTransSym(0)-ExoIMU2ExtentionY.getTransSym(0));
% AEEAttitudeExpr(3)=[1 0 0]*(ArmIMU2ExtentionZ.getTransSym(0)-ExoIMU2ExtentionZ.getTransSym(0));
AttachArmRotQuat=quatMultiply(quatConj(nhArm),ArmIMU2Frame.getAngSym(0));
AttachExoRotQuat=quatMultiply(quatConj(nhExo),ExoIMU2Frame.getAngSym(0));
AEEAttitudeExpr=[zeros(3,1),eye(3)]*quatMultiply(AttachArmRotQuat,quatConj(AttachExoRotQuat));
% AEEAttitudeExpr=[zeros(3,1),eye(3)]*quatMultiply(ArmIMU2Frame.getAngSym(0),quatConj(ExoIMU2Frame.getAngSym(0)));
AttachExoEndRx.setHCons(AEEAttitudeExpr(1),P_Sys.c);
AttachExoEndRy.setHCons(AEEAttitudeExpr(2),P_Sys.c);
AttachExoEndRz.setHCons(AEEAttitudeExpr(3),P_Sys.c);

% %% Kinematic Identification Use
% System.setCustomInfo({ArmAttachEnd 'ArmAttachEnd'; ExoAttachEnd 'ExoAttachEnd'});