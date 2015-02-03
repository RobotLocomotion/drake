function generateRandomBoxWorld(num_boxes, max_pose_array, max_size_array)
%NOTEST
%max_pose_array = [max_x, max_y] <-- goes from - max_x to + max_x
%max_size_array = [max_l, max_w, max_h] <-- goes from 0 to max_l

%set Gazebo model environment variable
clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',[fullfile(getDrakePath,'examples','Atlas','sdf')]); 

%create world xml
doc_node = com.mathworks.xml.XMLUtils.createDocument... 
    ('sdf')
doc_root_node = doc_node.getDocumentElement;
doc_root_node.setAttribute('version','1.4');

world_node = doc_node.createElement('world'); 
name_attribute = doc_node.createAttribute('name');
name_attribute.setNodeValue('random_terrain');
world_node.setAttributeNode(name_attribute);
doc_node.getDocumentElement.appendChild(world_node);

model_node = doc_node.createElement('model');
name_attribute = doc_node.createAttribute('name');
name_attribute.setNodeValue('block_field');
model_node.setAttributeNode(name_attribute);
world_node.appendChild(model_node);

static_node = doc_node.createElement('static');
static_node.appendChild... 
        (doc_node.createTextNode('true'));
model_node.appendChild(static_node);

pose_node = doc_node.createElement('pose');
pose_node.appendChild... 
        (doc_node.createTextNode('0 0 0 0 0 0'));
model_node.appendChild(pose_node);

link_node = doc_node.createElement('link');
name_attribute = doc_node.createAttribute('name');
name_attribute.setNodeValue('link');
link_node.setAttributeNode(name_attribute);
model_node.appendChild(link_node);

for i=0:num_boxes-1
    
    %create random numbers between -1 and 1
    rand_pose_array = (rand(1,2) * 2.0) - 1.0;
    rand_size_array = rand(1,3);
    rand_angle = (rand(1) * 2.0) - 1.0;
    %randomly distributed pose vals between + and - max_pose_array vals
    current_pose_array = (max_pose_array .* rand_pose_array);
    %randomly distributed size vals between 0 and max_size_array vals
    current_size_array = (max_size_array .* rand_size_array);
    current_angle = rand_angle * pi;
    
    visual_node = doc_node.createElement('visual');
    name_attribute = doc_node.createAttribute('name');
    name_attribute.setNodeValue(strcat('visual_',num2str(i)));
    visual_node.setAttributeNode(name_attribute);
    link_node.appendChild(visual_node);
    
    visual_pose_node = doc_node.createElement('pose');
    visual_pose_node.appendChild... 
        (doc_node.createTextNode(strcat(num2str(current_pose_array(1)),{' '},num2str(current_pose_array(2)), {' '}, num2str(current_size_array(3)/2.0), {' 0 0 '},num2str(current_angle))));
    visual_node.appendChild(visual_pose_node);
    
    visual_geometry_node = doc_node.createElement('geometry');
    visual_node.appendChild(visual_geometry_node);
    
    visual_box_node = doc_node.createElement('box');
    visual_geometry_node.appendChild(visual_box_node);
    
    visual_size_node = doc_node.createElement('size');
    visual_size_node.appendChild... 
        (doc_node.createTextNode(strcat(num2str(current_size_array(1)),{' '},num2str(current_size_array(2)),{' '},num2str(current_size_array(3)))));
    visual_box_node.appendChild(visual_size_node);
    
    collision_node = doc_node.createElement('collision');
    name_attribute = doc_node.createAttribute('name');
    name_attribute.setNodeValue(strcat('collision_',num2str(i)));
    collision_node.setAttributeNode(name_attribute);
    link_node.appendChild(collision_node);
    
    collision_pose_node = doc_node.createElement('pose');
    collision_pose_node.appendChild... 
        (doc_node.createTextNode(strcat(num2str(current_pose_array(1)),{' '},num2str(current_pose_array(2)),{' '},num2str(current_size_array(3)/2.0), {' 0 0 '},num2str(current_angle))));
    collision_node.appendChild(collision_pose_node);
    
    collision_geometry_node = doc_node.createElement('geometry');
    collision_node.appendChild(collision_geometry_node);
    
    collision_box_node = doc_node.createElement('box');
    collision_geometry_node.appendChild(collision_box_node);
    
    collision_size_node = doc_node.createElement('size');
    collision_size_node.appendChild... 
        (doc_node.createTextNode(strcat(num2str(current_size_array(1)),{' '},num2str(current_size_array(2)),{' '},num2str(current_size_array(3)))));
    collision_box_node.appendChild(collision_size_node);
    
end

world_node.appendChild(doc_node.createComment('A global light source'));

sun_include_node = doc_node.createElement('include');
world_node.appendChild(sun_include_node);

sun_uri_node = doc_node.createElement('uri');
sun_uri_node.appendChild... 
        (doc_node.createTextNode('model://sun'))
sun_include_node.appendChild(sun_uri_node);

world_node.appendChild(doc_node.createComment('A ground plane'));

ground_include_node = doc_node.createElement('include');
world_node.appendChild(ground_include_node);

ground_uri_node = doc_node.createElement('uri');
ground_uri_node.appendChild... 
        (doc_node.createTextNode('model://ground_plane'))
ground_include_node.appendChild(ground_uri_node);

ground_pose_node = doc_node.createElement('pose');
ground_pose_node.appendChild...
        (doc_node.createTextNode('0 0 0 0 0 0'));
ground_include_node.appendChild(ground_pose_node);


xmlFileName = ['sdf/random_terrain.world'];
xmlwrite(xmlFileName,doc_node);
type(xmlFileName);
