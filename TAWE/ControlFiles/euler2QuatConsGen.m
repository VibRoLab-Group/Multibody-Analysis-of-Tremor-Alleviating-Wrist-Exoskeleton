syms a1 a2 a3 a4 b1 b2 b3 b4 q1 q2 q3 real
quatCons=[zeros(3,1),eye(3)]*quatMultiply([a1;a2;a3;a4],quatConj([b1;b2;b3;b4]));
quatConsJac1=jacobian(quatCons,[a1,a2,a3,a4]);
quatInYPR=ypr2Quat([q1;q2;q3]);
quatConsJac2=jacobian(quatInYPR,[q1;q2;q3]);
% quatConsJac2Simp=sym(ones(4,3));
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(1))==0.5);
% quatConsJac2Simp(elementNum)=0.5*a1;
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(1))==-0.5);
% quatConsJac2Simp(elementNum)=-0.5*a1;
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(2))==0.5);
% quatConsJac2Simp(elementNum)=0.5*a2;
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(2))==-0.5);
% quatConsJac2Simp(elementNum)=-0.5*a2;
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(3))==0.5);
% quatConsJac2Simp(elementNum)=0.5*a3;
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(3))==-0.5);
% quatConsJac2Simp(elementNum)=-0.5*a3;
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(4))==0.5);
% quatConsJac2Simp(elementNum)=0.5*a4;
% elementNum=find(jSimplify(quatConsJac2/quatInYPR(4))==-0.5);
% quatConsJac2Simp(elementNum)=-0.5*a4;
% 

quatConsJac=quatConsJac1*quatConsJac2;
matlabFunction(quatConsJac,'file',"./ControlFiles/yprQuatConsJac.m",'vars',{[b1;b2;b3;b4],[q1;q2;q3]});