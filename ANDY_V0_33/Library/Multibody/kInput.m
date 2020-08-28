classdef kInput < kHVar
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kInput(inName,inSys,inDes)
            obj@kHVar(inName,inSys,inDes);
            inSys.Input.reg(inDes,obj);
        end
    end
end