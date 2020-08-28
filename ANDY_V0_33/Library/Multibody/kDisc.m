classdef kDisc < kHVar
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kDisc(inName,inSys,inDes)
            obj@kHVar(inName,inSys,inDes);
            inSys.Discrete.reg(inDes,obj);
        end
    end
end