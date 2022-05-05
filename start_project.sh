NAME=$(rosparam get name)
echo "<launch>" > project.launch
echo '	<node name="kf_scalar_filters" pkg="project" type="kf_scalar_filters.py" output="screen"/>' >> project.launch
echo '	<node name="local_occupancy_grid_map" pkg="project" type="local_occupancy_grid_map.py" output="screen"/>' >> project.launch
echo '	<node name="amcl_odom" pkg="project" type="amcl_odom.py" output="screen"/>' >> project.launch
echo '	<node name="turtle1_tf2_broadcaster" pkg="turtle_tf2" type="turtle_tf2_broadcaster.py" respawn="false" output="screen" >' >> project.launch
echo "		<param name="turtle" type="string" value="turtle1" />" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch
echo "" >> project.launch