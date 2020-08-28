function [TF,Vel,Cor,Jac,TransDis,Quat,MM,GFI,GFF,GFU,JacU,JacCons,CorCons,GFCons]=FUNCNAME_(t,q,p,u,s)
    
    [TF,Vel,Cor,Jac,TransDis,Quat]=KINEMATICS_(t,q,p,u,s);
    [MM,GFI]=INERTIAL_(t,q,p,u,s,TF,Vel,Cor,Jac);
    [GFF]=FORCE_(t,q,p,u,s,TF,TransDis,Vel,Jac,Quat);
    [GFU,JacU]=INPUT_(t,q,p,u,s,TF,TransDis,Vel,Jac,Quat);
    [JacCons,CorCons,GFCons]=CONSTRAINT_(t,q,p,u,s,TransDis,Vel,Cor,Jac,Quat);
