function [inertM,inertGF]=FUNCNAME_(t,q,p,u,s,TF_Global,Vel_Global,Cor_Global,Jac_Global)
    BaseFrameList=BASEFRAME__;
    MassList=MASS__(t,q,p,s,u);
    MomentList=reshape(MOMENT__(t,q,p,s,u),[3 3 numel(MassList)]);
    
    inertM=zeros(numel(q)/2,numel(q)/2,numel(MassList));
    inertGF=zeros(numel(q)/2,numel(MassList));


    for bodyNum=1:numel(MassList)
        rotor=TF_Global(1:3,1:3,BaseFrameList(bodyNum));

        m=MassList(bodyNum);
        I=rotor*MomentList(:,:,bodyNum)*rotor.';

        w=Vel_Global(4:6,BaseFrameList(bodyNum));
        a=Cor_Global(1:3,BaseFrameList(bodyNum));
        alpha=Cor_Global(4:6,BaseFrameList(bodyNum));

        vJacobian=Jac_Global(1:3,:,BaseFrameList(bodyNum));
        wJacobian=Jac_Global(4:6,:,BaseFrameList(bodyNum));

        inertM(:,:,bodyNum)=vJacobian.'*m*vJacobian+wJacobian.'*I*wJacobian;
        inertGF(:,bodyNum)=-vJacobian.'*m*a-wJacobian.'*(I*alpha+cross(w,I*w));
    end