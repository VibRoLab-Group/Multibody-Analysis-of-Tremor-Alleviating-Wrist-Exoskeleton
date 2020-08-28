function [dq,ds,l]=FUNCNAME_(flow,t,q,p,u,s,l)
%% Dynamic flow switching map (Toolbox Internal Use)
%% 
%% Robotics & Mechatronics Lab, Virginia Tech
%% Author: Jiamin Wang; Revised: 28-Jan-2019
    dq=q*0;
    ds=s*0;
%SWITCHCASE_