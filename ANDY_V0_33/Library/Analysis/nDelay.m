classdef nDelay < nVar
    properties(SetAccess=protected)
        delayVar;
        delayTime;
        
    end
    
    methods(Access=public)
        function obj=nDelay(inName,inSys,inDes,inDelayVar,inDelayTime)
            obj@nVar(inName,inSys,inDes);
            inSys.Input.reg(inDes,obj);
            
            obj.delayVar=inDelayVar;
            obj.delayTime=inDelayTime;
        end
        
        function output=getDelayJac(obj)
            [state]=obj.System.getStateVector();
            output=jacobian(obj.delayVar,state)*heaviside(obj.System.TimeVar-obj.delayTime);
        end
    end
end