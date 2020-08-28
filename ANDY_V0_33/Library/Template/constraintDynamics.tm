function [ConsJac,ConsCor,ConsGF]=FUNCNAME_(t,q,p,u,s,TransDis_Global,Vel_Global,Cor_Global,Jac_Global,Quat_Global)
%% Constraint dynamic property calculator (Toolbox Internal Use)
%% 
%% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
%% Author: Jiamin Wang; Revised: 28-Jan-2019

    ConsNum=CONSNUM_;
    if(ConsNum>0)
        ConsJac=zeros(ConsNum,numel(q)/2);
        ConsCor=zeros(ConsNum,1);
        ConsGF=zeros(ConsNum,1);
    else
        ConsJac=zeros(1,numel(q)/2);
        ConsCor=0;
        ConsGF=0;
    end
    
    SubConsJac=zeros(1,numel(q)/2);
    SubConsCor=0;
    SubConsGF=0;
    for ConsCount=1:ConsNum
        
%SWITCHCASE_
        
        ConsJac(ConsCount,:)=SubConsJac;
        ConsCor(ConsCount,:)=SubConsCor;
        ConsGF(ConsCount,:)=SubConsGF;
    end