clear;clc;close all;

res=[20, 20];
tsreduc=0.85;
passreduc=0.99;
maxmove=3;
nextstep={};
erob=[10,8;14,12];
alyrob=[10, 18];

pos=[4, 7];
net=[20, 10];

done=0;
% while ~done
    nextstep={};
    %figure()
    hold on
    %plot(pos(1),pos(2),'O',erob(:,1),erob(:,2),'rO',net(1), net(2),'x', alyrob(:,1), alyrob(:,2),'bO')
    xlim([0 20])
    ylim([0 20])
    
    cpos=pos;
    
    parray=[];
    nextstep{end+1}=pos;
    parray(end+1)=pcost(pos,erob);
    
    punisharray=1;
    
    [parray, nextstep, punisharray, num]=timestep({cpos},maxmove,erob,alyrob,tsreduc,passreduc,parray,nextstep,punisharray,res,0);
    disp(num)
    [parray, nextstep, punisharray, num]=timestep(nextstep,maxmove,erob,alyrob,tsreduc,passreduc,parray,nextstep,punisharray,res,num);
    disp(num)
%     [parray, nextstep, punisharray, num]=timestep(nextstep,maxmove,erob,alyrob,tsreduc,passreduc,parray,nextstep,punisharray,res,num);
%     disp(num)
    
    disp(parray(1))
    
    [M, I]=max(parray);
    disp(M)
    
    cla()
    plot(nextstep{I}(:,1),nextstep{I}(:,2),'-gO')
    plot([nextstep{I}(end,1),net(1)],[nextstep{I}(end,2), net(2)],'r--')
    plot(pos(1),pos(2),'O',erob(:,1),erob(:,2),'rO',net(1), net(2),'x', alyrob(:,1), alyrob(:,2),'bO')
    
    title('Robot action using Optimal Control')
    
%     pause(0.05)
    drawnow
    pos=(pos+nextstep{I}(2,:))/2;
    
    if I==1
        done=1;
    end
% end
function [parr, nstep, punisharray, count]=timestep(cpos,maxmove,erob,alyrob,tsreduc,passreduc,parr,nstep,punisharray,res,count)
% disp(count);
for ll=count+1:length(cpos)
    pos=cpos{ll}(end,:);
    newposx=[];
    newposy=[];
    anewposx=[];
    anewposy=[];
    for ii=-maxmove:maxmove:maxmove
        for jj=-maxmove:maxmove:maxmove
            xposnew=pos(1)+jj;
            yposnew=pos(2)+ii;
            if xposnew>0 && xposnew<res(1) && yposnew>0 && yposnew<res(2)
                newposx(end+1)=xposnew;
                newposy(end+1)=yposnew;
            end
        end
    end
    
    if length(cpos)>1
        for ii=-maxmove:maxmove:maxmove
            for jj=-maxmove:1:maxmove
                xposnew=alyrob(1)+jj;
                yposnew=alyrob(2)+ii;
                if xposnew>0 && xposnew<res(1) && yposnew>0 && yposnew<res(2)
                    anewposx(end+1)=xposnew;
                    anewposy(end+1)=yposnew;
                end
            end
        end
    end
    
    for kk=1:length(newposx)
        parr(end+1)=pcost([newposx(kk), newposy(kk)],erob)*tsreduc*punisharray(ll);
        nstep{end+1}=[cpos{ll};newposx(kk), newposy(kk)];
        punisharray(end+1)=punisharray(ll)*tsreduc;
    end
    for nn=1:length(anewposx)
        arob=[anewposx' anewposy'];
        for mm=1:length(arob(:,1))
            parr(end+1)=pcost(arob(mm,:),erob)*tsreduc*passreduc*passcost(pos,arob(mm,:),erob)*punisharray(ll);
            nstep{end+1}=[cpos{ll};arob(mm,:)];
            punisharray(end+1)=punisharray(ll)*tsreduc*passreduc*passcost(pos,arob(mm,:),erob);
        end
    end
    for mm=1:length(alyrob(:,1))
        parr(end+1)=pcost(alyrob(mm,:),erob)*tsreduc*passreduc*passcost(pos,alyrob(mm,:),erob)*punisharray(ll);
        nstep{end+1}=[cpos{ll};alyrob(mm,:)];
        punisharray(end+1)=punisharray(ll)*tsreduc*passreduc*passcost(pos,alyrob(mm,:),erob);
    end
end
count=ll;
end
function p=pcost(pos,erob)
maxdis=sqrt(10^2+20^2);
net=[20, 10];

dis=sqrt(sum(pos-net).^2);

p=1-0.5*dis/maxdis;

x=linspace(pos(1),net(1),30);
y=linspace(pos(2),net(2),30);

minde=min(sqrt((x-erob(:,1)).^2+(y-erob(:,2)).^2),[],2);

factor=1-1./minde;

for ii=1:length(factor)
    if factor(ii)<0
        factor(ii)=0;
    end
    p=p*factor(ii);
end

dif=net-pos;
angle=abs(atan(dif(2)/dif(1)))*180/pi;
factorangle=1-angle/90;

p=p*factorangle;

end

function p=passcost(start,endp,erob)
maxdis=sqrt(10^2+20^2);

dis=sqrt(sum(start-endp).^2);

p=1-0.4*dis/maxdis;

x=linspace(start(1),endp(1),30);
y=linspace(start(2),endp(2),30);

minde=min(sqrt((x-erob(:,1)).^2+(y-erob(:,2)).^2),[],2);

factor=1-1./minde;

for ii=1:length(factor)
    if factor(ii)<0
        factor(ii)=0;
    end
    p=p*factor(ii);
end

end