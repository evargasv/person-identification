function code_book = train_codebook( F, N )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

    k = 16;                  

    for i = 1:N

        mfcc_vec = F{i,1};
        lang_vec = F{i,3};
        train = mfcc_vec( lang_vec ~= 'E',: );
        
        
        
        if i == 13
            train = mfcc_vec( lang_vec == 'E',: );
        end
        
        code_book{i} = vqlbg(train', k);
        
        msg = sprintf('code book person %i ready', i);
        disp(msg);
    end

end

