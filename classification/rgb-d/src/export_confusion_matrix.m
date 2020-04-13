function export_confusion_matrix( C, N )
%EXPORT_CONFUSION_MATRIX Summary of this function goes here
%   Detailed explanation goes here

    for i=1:N    
        row = C(i,:);
        disp(sprintf('%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f&%0.2f \\\\',row));
        disp('\hline');
    end

end

