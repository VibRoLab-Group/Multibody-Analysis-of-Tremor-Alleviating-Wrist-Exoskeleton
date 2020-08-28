6Js=[eye(2) zeros(2,6)];
Jr=[zeros(6,2) eye(6)];

qwAmp=[35*pi/180;5*pi/12];
qwQmega=2*pi*[1;1]/2;
qwPhase=pi*[0;90]/180;

Iw=0.25*eye(2);
Kw=1*eye(2);
Bw=0.75*eye(2);

envFlex=65/180*pi;
envDev=32.5/180*pi;

biasDev=-7.5/180*pi;

Span1=linspace(-pi/2,pi/2,501);
Span2=linspace(pi/2,3*pi/2,501);

traj=[(envDev+biasDev)*cos(Span1),(envDev-biasDev)*cos(Span2);envFlex*sin(Span1),envFlex*sin(Span2)];

