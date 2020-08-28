function c=calInverseDynamics(c,p)
    exoParam=[p(20:22);p(32:34)]; %3 Link length and 3 init angles
    if exoParam(2)~=exoParam(3)
        error('Case not applicable');
    end
    %Can only be used for system with same link 2 and 2 length.
    armParam=[p(8:16);p(23:28);p(end)];
    EndPos=transKnownFunc(armParam,c);
    
    pureDist=EndPos(1:3)-[0;0;exoParam(1)];
    pureDistLength=norm(pureDist);
    trianglePeri=pureDistLength+exoParam(2)+exoParam(3);
    triangleArea=sqrt( trianglePeri *(trianglePeri - 2*pureDistLength) *(trianglePeri - 2*exoParam(2)) *(trianglePeri - 2*exoParam(3)) )/4;
    
    angle(3)= 2*acos((2*triangleArea/pureDistLength)/exoParam(2));
    angle(2)= (pi-angle(3))/2;
    angle(3)= -(pi-angle(3));
    
    pureHorizontalDist=norm(pureDist(1:2));
    tiltAngle=atan(pureDist(3)/pureHorizontalDist);
    angle(1)=-atan(pureDist(1)/pureDist(2));
    c(9)=angle(1)-exoParam(4);
    c(10)=angle(2)+tiltAngle-exoParam(5);
    c(11)=angle(3)-exoParam(6);
end