function [index_parameter, xparbase, rel_group,xparb_all] =minimal_model(w,xpar,filename,WRITE_FLAG)

tolerance_zero=1.e-4;
tolpal = length(w)*eps*max(abs(diag(w)));
tolzero = 1.e-4;
% tolerance_zero=1e-10;%1.e-4;
% tolpal = 8*length(w)*eps*max(abs(diag(w)));
% tolzero = 1e-10;%1.e-4;
% tolerance_qr=1.e-4;
tolpal=1.e-3;

ie = [];
ie = find(diag(w'*w)<tolzero);
ne = length(ie);
im = find(diag(w'*w)>tolzero);

wr = w(:,im);
xparr = xpar(im,:);

[q,r] = qr(wr,0);
ipb = find(abs(diag(r))>tolpal);
ipr = find(abs(diag(r))<=tolpal);
npb = length(ipb);
npr = length(ipr);
[mps, nps] = size(wr) ;
w1 = wr(:,ipb);
w2 = wr(:,ipr);
wp = [w1 w2];
[qp,rp] = qr(wp,0);
r1 = rp(1:npb,1:npb);
r2 = rp(1:npb,npb+1:nps);
b = inv(r1)*r2;

index_parameter = im(ipb) ;

% disp(['tolerance_zero : ', num2str(tolzero)])
% disp(['tolerance_qr : ', num2str(tolpal)])
% disp(' ');
% disp(['parametres elimines: ',num2str(ne)])
% disp(xpar(ie,:))
% disp(' ')
% disp(['parametres regroupes: ',num2str(npr)])
% disp(xparr(ipr,:))
% disp(' ')
% disp(['relation de regroupement: ',num2str(npb)])
xparb_all=[];
xparbase = [];
cstring=[];
cstring_neg=[];
rel_group=[];

if npb==nps
    return
else
    parb=1;
    for i=1:npb,
        
        xparb = xparr(ipb(i),:);
        %seuil pour parametres modifies, seuil_coef0=zero perturbe
        %detection des lignes nulles de b
        %si la ligne de b est nulle, le param?tre de base est inchange
        
        
        
        seuil_coef0 = tolerance_zero;	%zero perturb?pour les coefficients de b
        %NORM(V,inf) = max(abs(V))
        if norm(b(i,:),inf)>seuil_coef0,
            rel_group=[rel_group strvcat([deblank(xparb),'R= ' char(10)] )];
            
            xparbase = strvcat(xparbase, [deblank(xparb),'R']);
            xparb = [deblank(xparb),'R = ',xparr(ipb(i),:)];
            
            
            
            
            cstring=   [cstring '-(' strvcat(deblank([deblank(xparr(ipb(i),:)),'R+Tol(' num2str(i) '))+',deblank(xparr(ipb(i),:))])) ];
            cstring_neg=   [cstring_neg '(' strvcat(deblank([deblank(xparr(ipb(i),:)),'R-Tol(' num2str(i) '))-',deblank(xparr(ipb(i),:))])) ];
            
            % rel_group=[rel_group strvcat([deblank(xparr(ipb(i),:)),'R= ' char(10)] )];
            for j = 1:npr,
                %seuil pour coef de relation lin?aire =+1ou-1
                seuil_coef1 = tolerance_zero;
                if abs(b(i,j)-1)<seuil_coef1,
                    %on considere 1+seuil_coef1=1
                    xparb = [xparb,' + ',xparr(ipr(j),:)];
                    cstring=   [cstring,' + ',xparr(ipr(j),:)];
                    cstring_neg=   [cstring_neg,' - ',xparr(ipr(j),:)];
                    
                elseif abs(b(i,j)+1)<seuil_coef1,
                    %on considere -1+seuil_coef1=-1
                    xparb = [xparb,' - ',xparr(ipr(j),:)];
                    cstring=   [cstring,' - ',xparr(ipr(j),:)];
                    cstring_neg=   [cstring_neg,' + ',xparr(ipr(j),:)];
                elseif abs(b(i,j))>seuil_coef0,
                    %detection des coefficients nuls perturbes
                    %traitement du signe pas recuperable par num2str
                    if b(i,j) > 0,
                        xparb = [xparb,' + ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
                        cstring=   [cstring,' + ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
                        cstring_neg=   [cstring_neg,' - ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
                    else
                        xparb = [xparb,' - ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
                        cstring=   [cstring,' - ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
                        cstring_neg=   [cstring_neg,' + ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
                    end
                end
            end
            
            cstring=  ([cstring ';'  char(10)]);
            cstring_neg=   ([cstring_neg ';'  char(10)]);
            
            towrite=['c=[' cstring cstring_neg '];' ];
            
        else
            
            
            xparbase =strvcat(xparbase, deblank(xparb));
            rel_group=[rel_group strvcat([deblank(xparb),'= ' char(10)] )];
            
            %             cstring=[cstring '' strvcat(deblank([deblank(xparr(ipb(i),:)),'-Tol(' num2str(i) ')']))] ;
            %             cstring_neg=[cstring_neg '-' strvcat(deblank([deblank(xparr(ipb(i),:)),'+Tol(' num2str(i) ')']))] ;
        end
        
        %         cstring=  ([cstring ';'  char(10)]);
        %         cstring_neg=   ([cstring_neg ';'  char(10)]);
        %
        %         towrite=['c=[' cstring cstring_neg '];' ];
        xparb=[xparb ';'  char(10) ];
        
        xparb_all{parb}={xparb};
        disp([num2str(i) char(10) xparb ]);
        %             disp(xparb)
        % pause;clc
        parb=parb+1;
    end
end


%towrite


if nargin>2
    if WRITE_FLAG==1
        fwrite(fid, [towrite]);
        fclose(fid);
    end
end


diary off

%
% % function [index_parameter, xparbase] =...
% %   minimal_model(w,xpar)
% rel_group=0;
% tolerance_zero=1.e-6;
% tolpal = 1000*length(w)*eps*max(abs(diag(w)));
% tolzero = 1.e-8;
% tolerance_qr=1.e-4;
%
% ie = [];
% ie = find(diag(w'*w)<tolzero);
% ne = length(ie);
% im = find(diag(w'*w)>tolzero);
% wr = w(:,im);
% xparr = xpar(im,:);
%
% [q,r] = qr(wr,0);
% ipb = find(abs(diag(r))>tolpal);
% ipr = find(abs(diag(r))<=tolpal);
% npb = length(ipb);
% npr = length(ipr);
% [mps, nps] = size(wr) ;
% w1 = wr(:,ipb);
% w2 = wr(:,ipr);
% wp = [w1 w2];
% [qp,rp] = qr(wp,0);
% r1 = rp(1:npb,1:npb);
% r2 = rp(1:npb,npb+1:nps);
% b = inv(r1)*r2;
%
% index_parameter = im(ipb) ;
%
% disp(['tolerance_zero : ', num2str(tolzero)])
% disp(['tolerance_qr : ', num2str(tolpal)])
% disp(' ');
% disp(['parametres elimines: ',num2str(ne)])
% disp(xpar(ie,:))
% disp(' ')
% disp(['pramaetre regroupes: ',num2str(npr)])
% disp(xparr(ipr,:))
% disp(' ')
% disp(['relation regroupement: ',num2str(npb)])
%
% xparbase = [];
% if npb==nps
%    return
% else
%    for i=1:npb,
%       xparb = xparr(ipb(i),:);
%       %seuil pour param?tres modifi?s, seuil_coef0=z?ro perturb?
%       %d?tection des lignes nulles de b
%       %si la ligne de b est nulle, le param?tre de base est inchang?
%       seuil_coef0 = tolerance_zero;	%zero perturb?pour les coefficients de b
%       %NORM(V,inf) = max(abs(V))
%       if norm(b(i,:),inf)>seuil_coef0,
%          xparbase = strvcat(xparbase, [deblank(xparb),'R']);
%          xparb = [deblank(xparb),'R = ',xparr(ipb(i),:)];
%          for j = 1:npr,
%             %seuil pour coef de relation lin?aire =+1ou-1
%             seuil_coef1 = tolerance_zero;
%             if abs(b(i,j)-1)<seuil_coef1,
%                %on consid?re 1+seuil_coef1=1
%                xparb = [xparb,' + ',xparr(ipr(j),:)];
%             elseif abs(b(i,j)+1)<seuil_coef1,
%                %on consid?re -1+seuil_coef1=-1
%                xparb = [xparb,' - ',xparr(ipr(j),:)];
%             elseif abs(b(i,j))>seuil_coef0,
%                %d?tection des coefficients nuls perturb?s
%                %traitement du signe pas r?cup?rable par num2str
%                if b(i,j) > 0,
%                   xparb = [xparb,' + ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
%                else
%                   xparb = [xparb,' - ',num2str(abs(b(i,j))),'*',xparr(ipr(j),:)];
%                end
%             end
%          end
%       else
%          xparbase =strvcat(xparbase, deblank(xparb));
%       end
%       disp(xparb)
%    end
% end
% diary off
%
