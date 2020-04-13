function [X_Test, Y_Test, Y_Final] = test(testdir, n, code)
% Speaker Recognition: Testing Stage
%
% Input:
%       testdir : string name of directory contains all test sound files
%       n       : number of test files in testdir
%       code    : codebooks of all trained speakers
%
% Note:
%       Sound files in testdir is supposed to be:
%               s1.wav, s2.wav, ..., sn.wav
%
% Example:
%       >> test('C:\data\test\', 8, code);

X_Test = [];
Y_Test = [];
accuracy = zeros(15, 1);
accuracy_Final = zeros(15, 1);

yout = [];
Y_Final = [];

for k = 1:n                     % read test sound file of each speaker
    
    file = sprintf('%s%d.wav', testdir, k);
    [s_1, fs] = audioread(file);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Remove the noise:-
    [vs,zo]=vadsohn(s_1, fs, 'a');
    [r, c] = find(vs(:, 1)==1);
    
    s = zeros(size(r));
    for i = 1:size(r, 1)
        s(i) = s_1(r(i));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    v = mfcc(s, fs);            % Compute MFCC's
    v = mean(v, 2);
    
    %     X_Test = [X_Test; v'];
    
    Person = Test_Person_1(k);
    k_Temp = Person;% * ones(size(v, 1), 1);
    Y_Test = [Y_Test; k_Temp];
    
    %     for p = 1 : size(v, 1)
    
    distmin = inf;
    k1 = 0;
    
    for t = 1 : length(code)      % each trained codebook, compute distortion
        %             d = disteu(v(p, :), code{t});
        d = disteu(v', (code{t})');
        dist = sum(min(d,[],2)) / size(d,1);
        
        if dist < distmin
            distmin = dist;
            k1 = t;
        end
    end
    
    Person_1 = Test_Person(k1);
    yout = [yout; Person_1];
    %     end
    
    %     msg = sprintf('Speaker %d matches with speaker %d', k, k1);
    msg = sprintf('Speaker %d ', k);
    
    Y_Final = [Y_Final; yout];
    yout = [];
    
    %         if(1<=k && k<=9  &&  1<=k1 && k1<=6)
    %             accuracy(1) = accuracy(1)+1;
    %
    %         elseif(10<=k && k<=21  &&  7<=k1 && k1<=14)
    %             accuracy(2) = accuracy(2)+1;
    %
    %         elseif(22<=k && k<=30 &&  15<=k1 && k1<=20)
    %             accuracy(3) = accuracy(3)+1;
    %
    %         elseif(31<=k && k<=39  &&  21<=k1 && k1<=26 )
    %             accuracy(4) = accuracy(4)+1;
    %
    %         elseif(40<=k && k<=48   &&   27<=k1 && k1<=32)
    %             accuracy(5) = accuracy(5)+1;
    %
    %         elseif(49<=k && k<=57  &&  33<=k1 && k1<=38)
    %             accuracy(6) = accuracy(6)+1;
    %
    %         elseif (58<=k && k<=69   &&   39<=k1 && k1<=46)
    %             accuracy(7) = accuracy(7)+1;
    %
    %         elseif(70<=k && k<=78   &&   47<=k1 && k1<=52)
    %             accuracy(8) = accuracy(8)+1;
    %
    %         elseif(79<=k && k<=87   &&   53<=k1 && k1<=58)
    %             accuracy(9) = accuracy(9)+1;
    %
    %         elseif(88<=k && k<=96   &&   59<=k1 && k1<=64)
    %             accuracy(10) = accuracy(10)+1;
    %
    %         elseif(97<=k && k<=108    &&   65<=k1 && k1<=72)
    %             accuracy(11) = accuracy(11)+1;
    %
    %         elseif(109<=k && k<=117   &&   73<=k1 && k1<=78)
    %             accuracy(12) = accuracy(12)+1;
    %
    %         elseif(118<=k && k<=123   &&   79<=k1 && k1<=82)
    %             accuracy(13) = accuracy(13)+1;
    %
    %         elseif(124<=k && k<=135   &&  83<=k1 && k1<=90)
    %             accuracy(14) = accuracy(14)+1;
    %
    %         elseif(136<=k && k<=144   &&   91<=k1 && k1<=96)
    %             accuracy(15) = accuracy(15)+1;
    %         end
    
    %   if(1<=k && k<=70 &&   1<=k1 && k1<=70)
    %      accuracy(1) = accuracy(1)+1;
    % elseif(71<=k && k<=140 &&   71<=k1 && k1<=140)
    %    accuracy(2) = accuracy(2)+1;
    % elseif(141<=k && k<=215 && 141<=k1 && k1<=212)
    %    accuracy(3) = accuracy(3)+1;
    % elseif(216<=k && k<=282 && 213<=k1 && k1<=280)
    %    accuracy(4) = accuracy(4)+1;
    %elseif(283<=k && k<=352 && 281<=k1 && k1<=351)
    %  accuracy(5) = accuracy(5)+1;
    %elseif(353<=k && k<=423 && 352<=k1 && k1<=421)
    %    accuracy(6) = accuracy(6)+1;
    %elseif (424<=k && k<=505 && 422<=k1 && k1<=491)
    %   accuracy(7) = accuracy(7)+1;
    %elseif(506<=k && k<=564 && 492<=k1 && k1<=562)
    %   accuracy(8) = accuracy(8)+1;
    %elseif(565<=k && k<=640 && 563<=k1 && k1<=632)
    %   accuracy(9) = accuracy(9)+1;
    %elseif(641<=k && k<=705 && 633<=k1 && k1<=702)
    %   accuracy(10) = accuracy(10)+1;
    %elseif(706<=k && k<=776 && 703<=k1 && k1<=773)
    %   accuracy(11) = accuracy(11)+1;
    %end
    disp(msg);
    
end

%Total = [9, 12, 9, 9, 9, 9, 12, 9, 9, 9, 12, 9, 6, 12, 9];

% for i=1:15
%     disp('accuracy for speaker');
%     disp(i);
%     disp('is = ');
%     accuracy_Final(i) = accuracy(i) * 100 / Total(i);
%     disp(accuracy_Final(i));
% end


%accuracy_Final

end