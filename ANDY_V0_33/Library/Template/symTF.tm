function [frameTF]=FUNCNAME_(t,q,p,u,s)
    FrameNum=FRAMENUM_;
    
    frameTF=zeros(4,4,FrameNum);
    
    for ii=1:FrameNum
        frameTF(:,:,ii)=TFMAP_(ii,t,q,p,u,s);
    end