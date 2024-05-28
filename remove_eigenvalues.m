function [A] = remove_eigenvalues(A)
[vec,eigen] = eig(A);
for i = 1:size(eigen,1)
    if abs(eigen(i,i)) <= 10^-10
        A = A - 0.1*vec(:,i)*vec(:,i)'/norm(vec(:,i),2);
    end
end
end
