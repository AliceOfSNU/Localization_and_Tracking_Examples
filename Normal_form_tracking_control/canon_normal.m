function [An, Bn, Cn, Dn, T] = canon_normal(sys)

% Convert into Normal Form

[A,B,C,D] = ssdata(sys);
n = size(A,1);
r = length(pole(sys)) - length(zero(sys)) % relative degree of sys

T = C;
for i=1:r-1
    T = [T; C*A^i];
end

Ncandidate = null(B')';  % basis of left-null-space
N = [];
%for i=1:size(Ncandidate,1)
%    if rank( [T; Ncandidate(i,:)] ) > r
%        N = [N; Ncandidate(i,:)];
%    end
%    if rank([T; N]) == n
%        break
%    end
%end
N=[0 0 0 1];
T = [T; N];
disp(T)
An = T*A*inv(T);
Bn = T*B;
Cn = C*inv(T);
Dn = D;

end