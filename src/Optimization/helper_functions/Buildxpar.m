function xpar_vect = Buildxpar(frame_inertia,PLANAR,FRICTION)

    xpar_vect = [];
    NbSegment = length(frame_inertia);
    length_strpar = 2+length(num2str(frame_inertia(end)));
    
    switch nargin
        case 3
            if PLANAR == 1
                par = ['M ';'MX';'MY';'ZZ';];
            else
                par = ['M ';'MX';'MY';'MZ';'XY';'XZ';'YZ';'XX';'YY';'ZZ';];
            end
            
            if FRICTION == 1
                par = [par;'IA';'FS';'FV';];
            end
        case 2
            if PLANAR == 1
                par = ['M ';'MX';'MY';'ZZ';];
            else
                par = ['M ';'MX';'MY';'MZ';'XY';'XZ';'YZ';'XX';'YY';'ZZ';];
            end
            
        case 1
             par = ['M ';'MX';'MY';'MZ';'XY';'XZ';'YZ';'XX';'YY';'ZZ';];
             
    end
    

    for ii = 1:NbSegment
        for jj = 1:length(par)
            
            xpar = [deblank(par(jj,:)), num2str(frame_inertia(ii))];
            if length(xpar) < length_strpar
                xpar = [xpar,blanks(length_strpar-length(xpar))];
            end
            xpar_vect = [xpar_vect; xpar];
            
        end
    end
end