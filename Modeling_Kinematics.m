%% Declare Coordinate Frames in Forearm
[ArmIMU1Frame,ArmBaseDevFrame,ArmDevFrame,ArmDevFlexFrame,ArmFlexFrame,ArmIMU2Frame,ArmIMU2AdjustFrame....
]...
=System.Space.genNode({
                     'ArmIMU1Frame'
                     'ArmBaseDevFrame';
                     'ArmDevFrame';
                     'ArmDevFlexFrame';
                     'ArmFlexFrame';
                     'ArmIMU2Frame';
                     'ArmIMU2AdjustFrame'
                     });
                 
                 
[ArmBaseCOM,ArmWristCOM,ArmHandCOM]...
=System.Space.genNode({
                     'ArmBaseCOM';
                     'ArmWristCOM';
                     'ArmHandCOM';
                     });
                 

%% Declare Links in Forearm
World2AI1F=System.Space.genEdge({'World2AI1F' 'WORLD' 'ArmIMU1Frame'});
World2AI1F.setAngVel([q1rx.dot(1);q1ry.dot(1);q1rz.dot(1)],'q1BaseQuat').setTransDis([q1dx.dot(0);q1dy.dot(0);q1dz.dot(0)]).genProp;

AI1F2ABDF=System.Space.genEdge({'AI1F2ABDF' 'ArmIMU1Frame' 'ArmBaseDevFrame'});
ABDF2ADF=System.Space.genEdge({'ABDF2ADF' 'ArmBaseDevFrame' 'ArmDevFrame'});
ADF2ADFF=System.Space.genEdge({'ADF2ADFF' 'ArmDevFrame' 'ArmDevFlexFrame'});
ADFF2AFF=System.Space.genEdge({'ADFF2AFF' 'ArmDevFlexFrame' 'ArmFlexFrame'});
AFF2AI2F=System.Space.genEdge({'AFF2AI2F' 'ArmFlexFrame' 'ArmIMU2Frame'});
AFF2AI2AF=System.Space.genEdge({'AFF2AI2AF' 'ArmIMU2Frame' 'ArmIMU2AdjustFrame'});

q1DevAxisQuat=ypr2Quat([P_Init1.rf1Devx;P_Init1.rf1Devy;P_Init1.rf1Devz]);
AI1F2ABDF.setQuat(q1DevAxisQuat).setTransDis([P_Arm.l11x;P_Arm.l11y;P_Arm.l11z]).genProp;
% q1QuatTotal=q1DevAxisQuat;

q1DevQuat=ypr2Quat([P_Others.jointOrder*q1flex.dot(0);0;(1-P_Others.jointOrder)*q1dev.dot(0)]);
ABDF2ADF.setQuat(q1DevQuat).genProp;
% q1QuatTotal=quatMultiply(q1DevQuat,q1QuatTotal);

q1FlexAxisQuat=ypr2Quat([P_Init1.rf1Flexx;P_Init1.rf1Flexy;P_Init1.rf1Flexz]);
ADF2ADFF.setQuat(q1FlexAxisQuat).setTransDis([P_Arm.l12x;P_Arm.l12y;P_Arm.l12z]).genProp;
% q1QuatTotal=quatMultiply(q1FlexAxisQuat,q1QuatTotal);

q1FlexQuat=ypr2Quat([(1-P_Others.jointOrder)*q1flex.dot(0);0;P_Others.jointOrder*q1dev.dot(0)]);
ADFF2AFF.setQuat(q1FlexQuat).genProp;
% q1QuatTotal=quatMultiply(q1FlexQuat,q1QuatTotal);

% AFF2AI2F.setQuat(q1IMU2Quat).setTransDis([P_Arm.l13x;P_Arm.l13y;P_Arm.l13z]).genProp;
AFF2AI2F.setTransDis([P_Arm.l13x;P_Arm.l13y;P_Arm.l13z]).genProp;
% q1QuatTotal=quatMultiply(q1IMU2Quat,q1QuatTotal);

q1IMU2Quat=ypr2Quat([P_Init1.rf1IMU2x;P_Init1.rf1IMU2y;P_Init1.rf1IMU2z]);
AFF2AI2AF.setQuat(q1IMU2Quat).genProp;


AI1F2ABCOM=System.Space.genEdge({'AI1F2ABCOM' 'ArmIMU1Frame' 'ArmBaseCOM'});
ADF2AWCOM=System.Space.genEdge({'ADF2AWCOM' 'ArmDevFrame' 'ArmWristCOM'});
AI2AF2AHCOM=System.Space.genEdge({'AI2AF2AHCOM' 'ArmIMU2AdjustFrame' 'ArmHandCOM'});

AI1F2ABCOM.setTransDis([P_COM1.lm11x;P_COM1.lm11y;P_COM1.lm11z]).genProp;
ADF2AWCOM.setTransDis([P_COM1.lm12x;P_COM1.lm12y;P_COM1.lm12z]).genProp;
AI2AF2AHCOM.setTransDis([P_COM1.lm13x;P_COM1.lm13y;P_COM1.lm13z]).genProp;

%% Declare Coordinate Frames in Exoskeleton
[ExoIMU1Frame,ExoMedFrame,ExoPanFrame,ExoLink12Frame,ExoLink23Frame,ExoLink34Frame,ExoLink45Frame,...
    ExoAttachFrame,ExoIMU2Frame...
]...
=System.Space.genNode({
                     'ExoIMU1Frame'
                     'ExoMedFrame';
                     'ExoPanFrame'
                     'ExoLink12Frame'
                     'ExoLink23Frame'
                     'ExoLink34Frame'
                     'ExoLink45Frame'
                     'ExoAttachFrame'
                     'ExoIMU2Frame'
                     });
                 
                 
[ExoBaseCOM,ExoPanCOM,ExoLink1COM,ExoLink2COM,...
    ExoLink3COM,ExoLink4COM,ExoLinkJointCOM]...
=System.Space.genNode({
                     'ExoBaseCOM';
                     'ExoPanCOM';
                     'ExoLink1COM';
                     'ExoLink2COM';
                     'ExoLink3COM';
                     'ExoLink4COM';
                     'ExoLinkJointCOM';
                     });

                 
%% Declare Links in Exoskeleton
World2EI1F=System.Space.genEdge({'World2EI1F' 'WORLD' 'ExoIMU1Frame'});
World2EI1F.setAngVel([q2rx.dot(1);q2ry.dot(1);q2rz.dot(1)],'q2BaseQuat').setTransDis([q2dx.dot(0);q2dy.dot(0);q2dz.dot(0)]).genProp;
nhVec=System.getNHSignalVector;
nhArm=nhVec(1:4);
nhExo=nhVec(5:8);

EI1F2EMF=System.Space.genEdge({'EI1F2EMF' 'ExoIMU1Frame' 'ExoMedFrame'});
EMF2EPF=System.Space.genEdge({'EMF2EPF' 'ExoMedFrame' 'ExoPanFrame'});
EPF2EL12F=System.Space.genEdge({'EPF2EL12F' 'ExoPanFrame' 'ExoLink12Frame'});
EL12F2EL23F=System.Space.genEdge({'EL12F2EL23F' 'ExoLink12Frame' 'ExoLink23Frame'});
EL23F2EL34F=System.Space.genEdge({'EL23F2EL34F' 'ExoLink23Frame' 'ExoLink34Frame'});
EL34F2EL45F=System.Space.genEdge({'EL34F2EL45F' 'ExoLink34Frame' 'ExoLink45Frame'});
EL45F2EAF=System.Space.genEdge({'EL45F2EAF' 'ExoLink45Frame' 'ExoAttachFrame'});
EAF2EI2F=System.Space.genEdge({'EAF2EI2F' 'ExoAttachFrame' 'ExoIMU2Frame'});

q2MedAxisQuat=ypr2Quat([P_Init2.rf2Medx;P_Init2.rf2Medy;P_Init2.rf2Medz]);
EI1F2EMF.setQuat(q2MedAxisQuat).setTransDis([P_Exo.l21x;P_Exo.l21y;P_Exo.l21z]).genProp;
% q2QuatTotal=q2MedAxisQuat;

q2Rot01Quat=ypr2Quat([0;0;P_Init2.riw1+q2w1.dot(0)]);
EMF2EPF.setQuat(q2Rot01Quat).genProp;
% q2QuatTotal=quatMultiply(q2Rot01Quat,q2QuatTotal);

q2Rot12Quat=ypr2Quat([P_Init2.riw2+q2w2.dot(0);0;0]);
EPF2EL12F.setQuat(q2Rot12Quat).genProp;
% q2QuatTotal=quatMultiply(q2Rot12Quat,q2QuatTotal);

q2Rot23Quat=ypr2Quat([P_Init2.riw3+q2w3.dot(0);0;0]);
EL12F2EL23F.setQuat(q2Rot23Quat).setTransDis([0;P_Exo.l21;0]).genProp;
% q2QuatTotal=quatMultiply(q2Rot23Quat,q2QuatTotal);

q2Rot34Quat=ypr2Quat([P_Init2.riw4+q2w4.dot(0);0;0]);
EL23F2EL34F.setQuat(q2Rot34Quat).setTransDis([0;P_Exo.l22;0]).genProp;
% q2QuatTotal=quatMultiply(q2Rot34Quat,q2QuatTotal);

q2Rot45Quat=ypr2Quat([0;P_Init2.riw5+q2w5.dot(0);0]);
EL34F2EL45F.setQuat(q2Rot45Quat).setTransDis([0;P_Exo.l23;0]).genProp;
% q2QuatTotal=quatMultiply(q2Rot45Quat,q2QuatTotal);

q2Rot56Quat=ypr2Quat([0;0;P_Init2.riw6+q2w6.dot(0)]);
EL45F2EAF.setQuat(q2Rot56Quat).setTransDis([P_Exo.l24;0;P_Exo.l25]).genProp;
% q2QuatTotal=quatMultiply(q2Rot56Quat,q2QuatTotal);

q2IMU2Quat=ypr2Quat([P_Init1.rf1IMU2x;P_Init1.rf1IMU2y;P_Init1.rf1IMU2z]);
% q2AttQuat=ypr2Quat([P_Init1.rf1Attx;P_Init1.rf1Atty;P_Init1.rf1Attz]);
EAF2EI2F.setQuat(quatConj(q2IMU2Quat)).setTransDis([P_Arm.l14x;P_Arm.l14y;P_Arm.l14z]).genProp;
% q2QuatTotal=quatMultiply(q2AttQuat,q2QuatTotal);

EI1F2EBCOM=System.Space.genEdge({'EI1F2EBCOM' 'ExoIMU1Frame' 'ExoBaseCOM'});
EPF2EPCOM=System.Space.genEdge({'EPF2EPCOM' 'ExoPanFrame' 'ExoPanCOM'});
EL12F2EL1COM=System.Space.genEdge({'EL12F2EL1COM' 'ExoLink12Frame' 'ExoLink1COM'});
EL23F2EL2COM=System.Space.genEdge({'EL23F2EL2COM' 'ExoLink23Frame' 'ExoLink2COM'});
EL34F2EL3COM=System.Space.genEdge({'EL34F2EL3COM' 'ExoLink34Frame' 'ExoLink3COM'});
EL45F2EL4COM=System.Space.genEdge({'EL45F2EL4COM' 'ExoLink45Frame' 'ExoLink4COM'});
EAF2ELJCOM=System.Space.genEdge({'EAF2ELJCOM' 'ExoAttachFrame' 'ExoLinkJointCOM'});

EI1F2EBCOM.setTransDis([P_COM2.lm21x;P_COM2.lm21y;P_COM2.lm21z]).genProp;
EPF2EPCOM.setTransDis([P_COM2.lm22x;P_COM2.lm22y;P_COM2.lm22z]).genProp;
EL12F2EL1COM.setTransDis([P_COM2.lm23x;P_COM2.lm23y;P_COM2.lm23z]).genProp;
EL23F2EL2COM.setTransDis([P_COM2.lm24x;P_COM2.lm24y;P_COM2.lm24z]).genProp;
EL34F2EL3COM.setTransDis([P_COM2.lm25x;P_COM2.lm25y;P_COM2.lm25z]).genProp;
EL45F2EL4COM.setTransDis([P_COM2.lm26x;P_COM2.lm26y;P_COM2.lm26z]).genProp;
EAF2ELJCOM.setTransDis([P_COM2.lm27x;P_COM2.lm27y;P_COM2.lm27z]).genProp;
                 
%% Generate Kinematic System
System.Space.plotGraph(1);
System.Space.makeNumKinematics();