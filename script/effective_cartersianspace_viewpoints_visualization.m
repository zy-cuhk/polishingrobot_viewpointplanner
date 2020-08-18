clc,clear all, close all

wall2camera_distance=0.80;
camera_fov_length=wall2camera_distance/0.80*0.54;
camera_fov_width=camera_fov_length*2/3;

data=load('data.mat','camera_viewpoints','region_points');
effective_camera_viewpoints=data.camera_viewpoints;
region_points=data.region_points;
x_const=region_points(1,1);
ymin=min(region_points(:,2));
ymax=max(region_points(:,2));
zmin=min(region_points(:,3));
zmax=max(region_points(:,3));

figure(1);
for i=1:1:size(effective_camera_viewpoints,1)
    for j=1:1:size(effective_camera_viewpoints,2)
        flag=effective_camera_viewpoints(i,j,4);
        if flag==1
            ax=x_const;
            ay=ymin+(j-1)*camera_fov_width;
            az=zmin+(i-1)*camera_fov_length;
            a=[ax,ay,az];
            
            bx=x_const;
            by=ymin+(j-1)*camera_fov_width;
            bz=zmin+i*camera_fov_length;
            b=[bx,by,bz];
            
            cx=x_const;
            cy=ymin+j*camera_fov_width;
            cz=zmin+i*camera_fov_length;
            
            dx=x_const;
            dy=ymin+j*camera_fov_width;
            dz=zmin+(i-1)*camera_fov_length;
            
            if bz>zmax
                bz=zmax;
            end
            if cy>ymax
                cy=ymax;
            end
            if cz>zmax
                cz=zmax;
            end
            if dy>ymax
                dy=ymax;
            end
            
            %% draw the visiable region
            A=[ax;ay;az];
            B=[bx;by;bz];
            C=[cx;cy;cz];
            D=[dx;dy;dz];
            P=[B,A;C,D];
            X=P([1,4],:);
            Y=P([2,5],:);
            Z=P([3,6],:);
            h=surf(X,Y,Z);
            hold on;
            set(h,'FaceColor','b');
            
            viewpoint(:)=effective_camera_viewpoints(i,j,1:3);
            %% draw the first line
            line1(1,:)=A;
            line1(2,:)=viewpoint;
            plot3(line1(:,1),line1(:,2),line1(:,3),'r','LineWidth',2);
            hold on;
            %% draw the second line
            line2(1,:)=B;
            line2(2,:)=viewpoint;
            plot3(line2(:,1),line2(:,2),line2(:,3),'r','LineWidth',2);
            hold on;
            %% draw the third line
            line3(1,:)=C;
            line3(2,:)=viewpoint;
            plot3(line3(:,1),line3(:,2),line3(:,3),'r','LineWidth',2);
            hold on;            
            %% draw the third line
            line4(1,:)=D;
            line4(2,:)=viewpoint;
            plot3(line4(:,1),line4(:,2),line4(:,3),'r','LineWidth',2);
            hold on;     
        end
    end
end
axis equal;

% for i 

















