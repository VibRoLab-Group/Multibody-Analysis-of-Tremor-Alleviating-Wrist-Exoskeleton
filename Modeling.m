PathSetup;
PathSetup;

%% Declare Time Variable and System
t=sym('t','real');
System=kSystem('ForearmExoskeletonModel',t);
WORLD=System.Space.RootFrame;
System.Space.setPrecompile(true);
System.Model.setPrecompile(true);

%% Declare Constant Parameters
C=System.genParam({
             'empty' 'empty' 0; 
             });
                     
%% Declare Continuous States (Generalized Coordinates) 
[q1dev,q1flex,q1dx,q1dy,q1dz,q1rx,q1ry,q1rz]...
=System.genCont({
            'q1dev' 'Forearm Deviation Displacement';
            'q1flex' 'Forearm Flexion Displacement';
            'q1dx' 'Forearm X Translational Displacement';
            'q1dy' 'Forearm Y Translational Displacement';
            'q1dz' 'Forearm Z Translational Displacement';
            'q1rx' 'Forearm X Rotary Displacement';
            'q1ry' 'Forearm Y Rotary Displacement';
            'q1rz' 'Forearm Z Rotary Displacement';
            });
        
[q2w1,q2w2,q2w3,q2w4,q2w5,q2w6,q2dx,q2dy,q2dz,q2rx,q2ry,q2rz]...
=System.genCont({
            'q2w1' 'Exoskeleton Wrist Link Angle 1';
            'q2w2' 'Exoskeleton Wrist Link Angle 2';
            'q2w3' 'Exoskeleton Wrist Link Angle 3';
            'q2w4' 'Exoskeleton Wrist Link Angle 4';
            'q2w5' 'Exoskeleton Wrist Link Angle 5';
            'q2w6' 'Exoskeleton Wrist Link Angle 6';
            'q2dx' 'Exoskeleton X Translational Displacement';
            'q2dy' 'Exoskeleton Y Translational Displacement';
            'q2dz' 'Exoskeleton Z Translational Displacement';
            'q2rx' 'Exoskeleton X Rotary Displacement';
            'q2ry' 'Exoskeleton Y Rotary Displacement';
            'q2rz' 'Exoskeleton Z Rotary Displacement';
            });
        
%% Declare System Discrete Variables 
P_Sys=System.genDisc({
             'g' 'Gravity Acceleration';
             'c' 'Global Soft Constraint Factor';
             'b' 'Global Damping Factor';
             'b1dev' 'Deviation Damping Coefficient';
             'b1flex' 'Flexion Damping Coefficient';
             'b2act' 'Actuator Axis Damping Coefficient';
             'b2link' 'Angle Link Damping Coefficient';
             });
         
P_Arm=System.genDisc({
             'l11x' 'Deviation Joint Deviation along x from IMU1 axis (after rotation)';
             'l11y' 'Deviation Joint Deviation along y from IMU1 axis (after rotation)';
             'l11z' 'Deviation Joint Deviation along z from IMU1 axis (after rotation)';
             'l12x' 'Flexion Joint Deviation along x from deviation frame (after rotation)';
             'l12y' 'Flexion Joint Deviation along y from deviation frame (after rotation)';
             'l12z' 'Flexion Joint Deviation along z from deviation frame (after rotation)';
             'l13x' 'Hand IMU Deviation along x from flex frame (after rotation)';
             'l13y' 'Hand IMU Deviation along y from flex frame (after rotation)';
             'l13z' 'Hand IMU Deviation along z from flex frame (after rotation)';
             'l14x' 'Connection Point Deviation along x from IMU2 frame (after rotation)';
             'l14y' 'Connection Point Deviation along y from IMU2 frame (after rotation)';
             'l14z' 'Connection Point Deviation along z from IMU2 frame (after rotation)';
             });
         
P_Exo=System.genDisc({
             'l21x' 'Exo Device Distance along x from IMU1 axis';
             'l21y' 'Exo Device Distance along y from IMU1 axis';
             'l21z' 'Exo Device Distance along z from IMU1 axis';   
             'l21' 'Linkage 1 Length';
             'l22' 'Linkage 2 Length';
             'l23' 'Linkage 3 Length';
             'l24' 'Linkage 4 Length';
             'l25' 'Linkage 5 Length';
             });
         
P_Init1=System.genDisc({
             'rf1Devx' 'Initial Deviation Axis (Z) Orientation x with respect to rotated supination axis';
             'rf1Devy' 'Initial Deviation Axis (Z) Orientation y with respect to rotated supination axis';
             'rf1Devz' 'Initial Deviation Axis (Z) Orientation z with respect to rotated supination axis';
             'rf1Flexx' 'Initial Flexion Axis (X) Orientation x with respect to rotated Deviation axis';
             'rf1Flexy' 'Initial Flexion Axis (X) Orientation y with respect to rotated Deviation axis';
             'rf1Flexz' 'Initial Flexion Axis (X) Orientation z with respect to rotated Deviation axis';
             'rf1IMU2x' 'Initial IMU2 Orientation x with respect to rotated flex axis';
             'rf1IMU2y' 'Initial IMU2 Orientation y with respect to rotated flex axis';
             'rf1IMU2z' 'Initial IMU2 Orientation z with respect to rotated flex axis';
             });
         
P_Init2=System.genDisc({
             'rf2Medx' 'Initial Device Orientation x (Forearm)';
             'rf2Medy' 'Initial Device Orientation y (Forearm)';
             'rf2Medz' 'Initial Device Orientation z (Forearm)';
             'riw1' 'w1 Initial Displacement';
             'riw2' 'w2 Initial Displacement';
             'riw3' 'w3 Initial Displacement';
             'riw4' 'w4 Initial Displacement';
             'riw5' 'w5 Initial Displacement';
             'riw6' 'w6 Initial Displacement';
             });
         
P_COM1=System.genDisc({
             'lm11x' 'Forearm Base COM x';
             'lm11y' 'Forearm Base COM y';
             'lm11z' 'Forearm Base COM z';
             'lm12x' 'Wrist COM x';
             'lm12y' 'Wrist COM y';
             'lm12z' 'Wrist COM z';
             'lm13x' 'Hand COM x';
             'lm13y' 'Hand COM y';
             'lm13z' 'Hand COM z';
             });
         
P_COM2=System.genDisc({
             'lm21x' 'Exoskeleton Base COM x';
             'lm21y' 'Exoskeleton Base COM y';
             'lm21z' 'Exoskeleton Base COM z';
             'lm22x' 'Exoskeleton Pan COM x';
             'lm22y' 'Exoskeleton Pan COM y';
             'lm22z' 'Exoskeleton Pan COM z';
             'lm23x' 'Exoskeleton Link 1 COM x';
             'lm23y' 'Exoskeleton Link 1 COM y';
             'lm23z' 'Exoskeleton Link 1 COM z';
             'lm24x' 'Exoskeleton Link 2 COM x';
             'lm24y' 'Exoskeleton Link 2 COM y';
             'lm24z' 'Exoskeleton Link 2 COM z';
             'lm25x' 'Exoskeleton Link 3 COM x';
             'lm25y' 'Exoskeleton Link 3 COM y';
             'lm25z' 'Exoskeleton Link 3 COM z';
             'lm26x' 'Exoskeleton Link 4 COM x';
             'lm26y' 'Exoskeleton Link 4 COM y';
             'lm26z' 'Exoskeleton Link 4 COM z';
             'lm27x' 'Exoskeleton Link Joint COM x';
             'lm27y' 'Exoskeleton Link Joint COM y';
             'lm27z' 'Exoskeleton Link Joint COM z';
             });
         
P_Inert1=System.genDisc({
             'm11' 'Arm Base Mass';
             'i11xx' 'Arm Base Inertia xx';
             'i11xy' 'Arm Base Inertia xy';
             'i11xz' 'Arm Base Inertia xz';
             'i11yy' 'Arm Base Inertia yy';
             'i11yz' 'Arm Base Inertia yz';
             'i11zz' 'Arm Base Inertia zz';
             'm12' 'Wrist Mass';
             'i12xx' 'Wrist Inertia xx';
             'i12xy' 'Wrist Inertia xy';
             'i12xz' 'Wrist Inertia xz';
             'i12yy' 'Wrist Inertia yy';
             'i12yz' 'Wrist Inertia yz';
             'i12zz' 'Wrist Inertia zz';
             'm13' 'Hand Mass';
             'i13xx' 'Hand Inertia xx';
             'i13xy' 'Hand Inertia xy';
             'i13xz' 'Hand Inertia xz';
             'i13yy' 'Hand Inertia yy';
             'i13yz' 'Hand Inertia yz';
             'i13zz' 'Hand Inertia zz';
             });
        
P_Inert2=System.genDisc({
             'm21' 'Exoskeleton Base Mass';
             'i21xx' 'Exoskeleton Base Inertia xx';
             'i21xy' 'Exoskeleton Base Inertia xy';
             'i21xz' 'Exoskeleton Base Inertia xz';
             'i21yy' 'Exoskeleton Base Inertia yy';
             'i21yz' 'Exoskeleton Base Inertia yz';
             'i21zz' 'Exoskeleton Base Inertia zz';
             'm22' 'Exoskeleton Pan Mass';
             'i22xx' 'Exoskeleton Pan Inertia xx';
             'i22xy' 'Exoskeleton Pan Inertia xy';
             'i22xz' 'Exoskeleton Pan Inertia xz';
             'i22yy' 'Exoskeleton Pan Inertia yy';
             'i22yz' 'Exoskeleton Pan Inertia yz';
             'i22zz' 'Exoskeleton Pan Inertia zz';
             'm23' 'Exoskeleton Link 1 Mass';
             'i23xx' 'Exoskeleton Link 1 Inertia xx';
             'i23xy' 'Exoskeleton Link 1 Inertia xy';
             'i23xz' 'Exoskeleton Link 1 Inertia xz';
             'i23yy' 'Exoskeleton Link 1 Inertia yy';
             'i23yz' 'Exoskeleton Link 1 Inertia yz';
             'i23zz' 'Exoskeleton Link 1 Inertia zz';
             'm24' 'Exoskeleton Link 2 Mass';
             'i24xx' 'Exoskeleton Link 2 Inertia xx';
             'i24xy' 'Exoskeleton Link 2 Inertia xy';
             'i24xz' 'Exoskeleton Link 2 Inertia xz';
             'i24yy' 'Exoskeleton Link 2 Inertia yy';
             'i24yz' 'Exoskeleton Link 2 Inertia yz';
             'i24zz' 'Exoskeleton Link 2 Inertia zz';
             'm25' 'Exoskeleton Link 3 Mass';
             'i25xx' 'Exoskeleton Link 3 Inertia xx';
             'i25xy' 'Exoskeleton Link 3 Inertia xy';
             'i25xz' 'Exoskeleton Link 3 Inertia xz';
             'i25yy' 'Exoskeleton Link 3 Inertia yy';
             'i25yz' 'Exoskeleton Link 3 Inertia yz';
             'i25zz' 'Exoskeleton Link 3 Inertia zz';
             'm26' 'Exoskeleton Link 4 Mass';
             'i26xx' 'Exoskeleton Link 4 Inertia xx';
             'i26xy' 'Exoskeleton Link 4 Inertia xy';
             'i26xz' 'Exoskeleton Link 4 Inertia xz';
             'i26yy' 'Exoskeleton Link 4 Inertia yy';
             'i26yz' 'Exoskeleton Link 4 Inertia yz';
             'i26zz' 'Exoskeleton Link 4 Inertia zz';
             'm27' 'Exoskeleton Link Joint Mass';
             'i27xx' 'Exoskeleton Link Joint Inertia xx';
             'i27xy' 'Exoskeleton Link Joint Inertia xy';
             'i27xz' 'Exoskeleton Link Joint Inertia xz';
             'i27yy' 'Exoskeleton Link Joint Inertia yy';
             'i27yz' 'Exoskeleton Link Joint Inertia yz';
             'i27zz' 'Exoskeleton Link Joint Inertia zz';
             });
         
P_Others=System.genDisc({
             'consExtLink' 'Constraint Extension Link at Attach Frame'
             'l1FlexShift' 'Shift of Flexion Axis along y Direction';
             'jointOrder' 'Set to 0 for RUD-FE, 1 for FE-RUD'
             });

 U1=System.genInput({
             'u1dev' 'Arm Deviation Input';
             'u1flex' 'Arm Flexion Input';
             'u1dx' 'Arm Float Base Translational x input';
             'u1dy' 'Arm Float Base Translational y input';
             'u1dz' 'Arm Float Base Translational z input';
             'u1rx' 'Arm Float Base Rotational x input';
             'u1ry' 'Arm Float Base Rotational y input';
             'u1rz' 'Arm Float Base Rotational z input';
             });

 U2=System.genInput({
             'u21' 'Exo Skeleton Input 1';
             'u22' 'Exo Skeleton Input 2';
             });
        
%% Kinematic and Dynamic Modeling
Modeling_Kinematics;
Modeling_Dynamics;

%% Compile Dynamics
System.Model.Init();
[FixArm,FloatArm]=System.Model.genNode({'FixArm','FloatArm'});
System.Model.plotGraph(2);
FixArm.genFlowEOM({'LockArmBaseDx','LockArmBaseDy','LockArmBaseDz',...
                    'LockArmBaseRx','LockArmBaseRy','LockArmBaseRz',...
                    'AttachExoBaseDx','AttachExoBaseDy','AttachExoBaseDz',...
                    'AttachExoBaseRx','AttachExoBaseRy','AttachExoBaseRz',...
                    'AttachExoEndDx','AttachExoEndDy','AttachExoEndDz',...
                   'AttachExoEndRx','AttachExoEndRy','AttachExoEndRz',...
                    }.');
FloatArm.genFlowEOM({...
                   'AttachExoBaseDx','AttachExoBaseDy','AttachExoBaseDz',...
                   'AttachExoBaseRx','AttachExoBaseRy','AttachExoBaseRz',...
                   'AttachExoEndDx','AttachExoEndDy','AttachExoEndDz',...
                   'AttachExoEndRx','AttachExoEndRy','AttachExoEndRz',...
                   }.');
System.Model.makeDynamics('num');