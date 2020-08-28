classdef nState < nTimeVar
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=nState(inName,inSys,inDes,inOrder)
            obj@nTimeVar(inName,inSys,inDes,inOrder);
            inSys.State.reg(obj);
        end
         
        function obj=setInitVal(obj,inInitVal)
            if(numel(inInitVal)~=obj.Order)
                error(obj.msgStr('Error','The input and state must have the same order!'));
            else
                obj.InitVal=reshape(inInitVal,[],1);
            end
        end
    end
end