classdef jSys < jObj
% jSys is the basis for all the system objects, such as kSystem and
% nContSys
    methods(Access=public)
        function obj=jSys(inName)
            obj@jObj(inName);
        end
    end
end