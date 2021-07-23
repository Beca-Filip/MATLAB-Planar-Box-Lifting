addpath('helper_functions\')
% Column names
xpar = [];
% for ii = 1 : Ncf
%     xpar = [xpar, strcat("C00", num2str(ii))];
% end
% for ii = 1 : m
%     if ii < 10
%         xpar = [xpar, strcat("L00", num2str(ii))];
%     end    
%     if ii >= 10 && ii < 100
%         xpar = [xpar, strcat("L0", num2str(ii))];
%     end
% end
% for ii = 1 : p
%     if ii < 10
%         xpar = [xpar, strcat("M00", num2str(ii))];
%     end
%     if ii >= 10 && ii < 100
%         xpar = [xpar, strcat("M0", num2str(ii))];
%     end
% end
for ii = 1 : Ncf
    xpar = [xpar; strcat('C00', num2str(ii))];
end
for ii = 1 : m
    if ii < 10
        xpar = [xpar; strcat('L00', num2str(ii))];
    end    
    if ii >= 10 && ii < 100
        xpar = [xpar; strcat('L0', num2str(ii))];
    end
end
for ii = 1 : p
    if ii < 10
        xpar = [xpar; strcat('M00', num2str(ii))];
    end
    if ii >= 10 && ii < 100
        xpar = [xpar; strcat('M0', num2str(ii))];
    end
    if ii >= 100 && ii < 1000
        xpar = [xpar; strcat('M', num2str(ii))];
    end
end

[index_parameter, xparbase, rel_group,xparb_all] = minimal_model(C,xpar, 'blabla', 0)