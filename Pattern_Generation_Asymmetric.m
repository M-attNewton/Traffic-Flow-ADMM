function [ K_Pattern ] = Pattern_Generation_Asymmetric( N,AV_number,CR_front,CR_rear )
switch AV_number
    case 1
        K_Pattern = zeros(1,2*N);
        for i = 1:CR_rear
            K_Pattern(1,2*i-1:2*i) = [1,1];
        end
        for i = N-CR_front : N-1
            K_Pattern(1,2*i-1:2*i) = [1,1];
        end
        K_Pattern(1,2*N-1:2*N) = [1,1];
%     case 2
%         if CR>=N-floor(N/2)
%             K_Pattern = ones(2,2*N);
%         else
%             K_Pattern = zeros(2,2*N);
%             % row 1
%             for i = floor(N/2)-CR : floor(N/2)+CR
%                 K_Pattern(1,2*i-1:2*i) = [1,1];
%             end
%             % row 2
%             
%             for i = 1:CR
%                 K_Pattern(2,2*i-1:2*i) = [1,1];
%             end
%             for i = N-CR : N-1
%                 K_Pattern(2,2*i-1:2*i) = [1,1];
%             end
%             K_Pattern(2,2*N-1:2*N) = [1,1];
%         end
        
end

end

