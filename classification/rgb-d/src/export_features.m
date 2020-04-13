function export_features( F, N )
%EXPORT_LATEX Summary of this function goes here
%   Detailed explanation goes here

    % export features
    for i=1:N
        f = F{i,2};
        n = uint8( size(f,1) );
        ns = size(f(f(:)=='S'),1);
        nw = size(f(f(:)=='W'),1);
        disp(sprintf('%i & %i & %i & %i \\\\',i,n,ns,nw));
        disp('\hline');
    end
    
end

