clc,clear all, close all
%% the setting parameters are:
wall2base_distance=0.80;
wall2camera_distance=0.80;

figure(1);
%% manipulator parameters are shown as follows:
manipulator_3dworkspace_radius=1.50;
manipulator_minheight=1.2;
manipulator_maxheight=1.6;
%% visualization of manipulator base positions in the global coordinate frame
manipulator_baseposition(1,:)=[0,0,manipulator_minheight];
manipulator_baseposition(2,:)=[0,0,manipulator_maxheight];
scatter3(manipulator_baseposition(:,1),manipulator_baseposition(:,2),manipulator_baseposition(:,3),'k');
hold on;

%% wall parameters are shown as follows:
wall_height=2.7;
%% visualization of wall structure

%% the intersection region between manipulator 3d workspace and wall surface are shown 
%% as follows: 
%% the circle centriod in the minimum height and maximum height, the circular radius
circle_centroids=zeros(2,3);
circle_centroids(1,1)=wall2base_distance;
circle_centroids(1,2)=0;
circle_centroids(1,3)=manipulator_minheight;
circle_centroids(1,1)=wall2base_distance;
circle_centroids(1,2)=0;
circle_centroids(1,3)=manipulator_maxheight;
circle_radius=sqrt(manipulator_3dworkspace_radius^2-wall2base_distance^2);

%% the intersection region visualization as follows:
t=0:0.1:(2*pi);
n=[t,0];
plot3(wall2base_distance*ones(size(n)), 0+circle_radius*sin(n),manipulator_minheight+circle_radius*cos(n));
hold on;
plot3(wall2base_distance*ones(size(n)), 0+circle_radius*sin(n),manipulator_maxheight+circle_radius*cos(n));
hold on;

%% the intersection points between higher-height circle and wall-ceiling
%% line are: point_a, point_b
costheta=(wall_height-manipulator_maxheight)/circle_radius;
point_a_y=-circle_radius*sqrt(1-costheta^2);
point_b_y=circle_radius*sqrt(1-costheta^2);
%% the intersection points betwen lower-height circle and wall-ground line
costheta=(-manipulator_minheight)/circle_radius;
point_c_y=-circle_radius*sqrt(1-costheta^2);
point_d_y=circle_radius*sqrt(1-costheta^2);
%% the intersection points are recorded as follows:
intersection_points=ones(4,3);
%% point a
intersection_points(1,1)=wall2base_distance;
intersection_points(1,2)=point_a_y;
intersection_points(1,3)=wall_height;
%% point b
intersection_points(2,1)=wall2base_distance;
intersection_points(2,2)=point_b_y;
intersection_points(2,3)=wall_height;
%% point c
intersection_points(3,1)=wall2base_distance;
intersection_points(3,2)=point_c_y;
intersection_points(3,3)=0;
%% point d
intersection_points(4,1)=wall2base_distance;
intersection_points(4,2)=point_d_y;
intersection_points(4,3)=0;
%% plot point a,b,c,d
scatter3(intersection_points(:,1),intersection_points(:,2),intersection_points(:,3),'k');
hold on;
%% the polishing region is defined as follows:
sidelength1=abs(point_b_y-point_a_y);
sidelength2=abs(point_c_y-point_d_y);
region_points=intersection_points;
if sidelength1<sidelength2
    region_points(3,2)=point_a_y;
    region_points(4,2)=point_b_y;
else
    region_points(1,2)=point_c_y;
    region_points(2,2)=point_d_y;
end
region_height=wall_height;
region_width=min(sidelength1,sidelength2);
%% plot the polishing region as follows:
line1(1,:)=region_points(1,:);
line1(2,:)=region_points(2,:);
plot3(line1(:,1),line1(:,2),line1(:,3),'b','LineWidth',2);
hold on;
line2(1,:)=region_points(2,:);
line2(2,:)=region_points(4,:);
plot3(line2(:,1),line2(:,2),line2(:,3),'b','LineWidth',2);
hold on;
line3(1,:)=region_points(4,:);
line3(2,:)=region_points(3,:);
plot3(line3(:,1),line3(:,2),line3(:,3),'b','LineWidth',2);
hold on;
line4(1,:)=region_points(3,:);
line4(2,:)=region_points(1,:);
plot3(line4(:,1),line4(:,2),line4(:,3),'b','LineWidth',2);
hold on;

%% camera parameters are shown as follows:
camera_fov_length=wall2camera_distance/0.80*0.54;
camera_fov_width=camera_fov_length*2/3;
%% camera viewpoints are shown as follows:
M=ceil(region_height/camera_fov_length);
N=ceil(region_width/camera_fov_width);
%% the first step is to generate the positions of horizontal intersection points and vertical intersection points
for i=1:1:M
    horizontal_intersectionpoints(2*i-1,1)=wall2base_distance;
    horizontal_intersectionpoints(2*i-1,2)=-region_width/2;
    horizontal_intersectionpoints(2*i-1,3)=i*camera_fov_length;
    line1(1,:)=horizontal_intersectionpoints(2*i-1,:);
    
    horizontal_intersectionpoints(2*i,1)=wall2base_distance;
    horizontal_intersectionpoints(2*i,2)=region_width/2;
    horizontal_intersectionpoints(2*i,3)=i*camera_fov_length;
    line1(2,:)=horizontal_intersectionpoints(2*i,:);
    
    plot3(line1(:,1),line1(:,2),line1(:,3),'b','LineWidth',2);
    hold on;
end

for i=1:1:N-1
    vertical_intersectionpoints(2*i-1,1)=wall2base_distance;
    vertical_intersectionpoints(2*i-1,2)=-region_width/2+i*camera_fov_width;
    vertical_intersectionpoints(2*i-1,3)=region_height; 
    line1(1,:)=vertical_intersectionpoints(2*i-1,:);
    
    vertical_intersectionpoints(2*i,1)=wall2base_distance;
    vertical_intersectionpoints(2*i,2)=-region_width/2+i*camera_fov_width;
    vertical_intersectionpoints(2*i,3)=0;
    line1(2,:)=vertical_intersectionpoints(2*i,:);
    
    plot3(line1(:,1),line1(:,2),line1(:,3),'b','LineWidth',2);
    hold on;
end

axis equal;


%% the second step is to generate the positions of viewpoint
camera_viewpoints=zeros(M,N,3);
count=1;
for i=1:1:M
    for j=1:1:N
        camera_viewpoints(i,j,1)=wall2base_distance-wall2camera_distance;
        camera_viewpoints(i,j,2)=-region_width/2+camera_fov_width*(j-0.5);
        camera_viewpoints(i,j,3)=camera_fov_length*i-camera_fov_length/2; 
        camera_viewpoints1(count,:)=camera_viewpoints(i,j,:);
        count=count+1;
    end
end
%% the visualization of camera viewpoints are shown as follows:
scatter3(camera_viewpoints1(:,1),camera_viewpoints1(:,2),camera_viewpoints1(:,3),'r');
axis([0 1 -1 1 0 3])
view(-90,0)
axis equal;

%% verification of camera viewpoints in the manipulator base frame
manipulator_lowerbase(:)=manipulator_baseposition(1,:);
manipulator_upperbase(:)=manipulator_baseposition(2,:);
manipulator_3dworkspace_radius=1.50;

count=1;
for i=1:1:size(camera_viewpoints1,1)
    deltax=abs(camera_viewpoints1(i,1)-manipulator_lowerbase(1));
    deltay=abs(camera_viewpoints1(i,2)-manipulator_lowerbase(2));
    deltaz=abs(camera_viewpoints1(i,3)-manipulator_lowerbase(3));
    delta=sqrt(deltax^2+deltay^2+deltaz^2);
    if delta<manipulator_3dworkspace_radius
        lowerbase_camera_viewpoints(count,:)=camera_viewpoints1(i,:);
        count=count+1;
    end
end
%% transform the global cooridate frame position to manipulator base frame
if size(lowerbase_camera_viewpoints,1)==M*N
    camera_viewpoints_inlowerbaseframe(:,1)=lowerbase_camera_viewpoints(:,1)-manipulator_lowerbase(1);
    camera_viewpoints_inlowerbaseframe(:,2)=lowerbase_camera_viewpoints(:,2)-manipulator_lowerbase(2);
    camera_viewpoints_inlowerbaseframe(:,3)=lowerbase_camera_viewpoints(:,3)-manipulator_lowerbase(3);
end

%% the visualization of camera viewpoints are shown as follows:
figure(2);
scatter3(camera_viewpoints_inlowerbaseframe(:,1),camera_viewpoints_inlowerbaseframe(:,2),camera_viewpoints_inlowerbaseframe(:,3),'r');
axis equal;

save('data2.mat','camera_viewpoints_inlowerbaseframe')