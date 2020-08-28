classdef kNHSig < kNHVar
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kNHSig(inName,inSys,inDes,inOrder)
            obj@kNHVar(inName,inSys,inDes,inOrder);
            inSys.NHSignal.reg(obj);
        end
        
        function obj=setInitVal(obj,inInitVal)
            if(numel(inInitVal)~=obj.Order)
                error(obj.msgStr('Error','The kCoord must be of the same order!'));
            else
                obj.InitVal=reshape(inInitVal,[],1);
            end
        end
    end
end