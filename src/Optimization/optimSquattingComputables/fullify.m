function varargout = fullify(f, varargin)

%FULLIFY makes a function output dense matrices.
[varargout{1:nargout}] = f(varargin{:});
for i=1:numel(varargout)
varargout{i} = full(varargout{i});
end

end