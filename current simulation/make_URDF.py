import file_handling
import os

def write_URDF_basic(urdf_str,indent,count,x,height,y,mass,friction):
    z=height
    urdf_str += "\t" * indent + "<link name = \"box" + str(count) + "\">\n"
    indent += 1

    urdf_str += "\t" * indent + "<visual>\n"
    indent +=1
    urdf_str += "\t" * indent + "<origin xyz=\"" + str(x) + " " + str(y) + " " + str(z) + "\"/>\n"
    urdf_str += "\t" * indent + "<geometry>\n"
    indent +=1
    urdf_str += "\t" * indent + "<box size = \"1 1 2\"/>\n"
    indent -=1
    urdf_str += "\t" * indent + "</geometry>\n"
    indent -=1
    urdf_str += "\t" * indent + "</visual>\n"

    urdf_str += "\t" * indent + "<collision>\n"
    indent +=1
    urdf_str += "\t" * indent + "<origin xyz=\"" + str(x) + " " + str(y) + " " + str(z) + "\"/>\n"
    urdf_str += "\t" * indent + "<geometry>\n"
    indent +=1
    urdf_str += "\t" * indent + "<box size = \"1 1 2\"/>\n"
    indent -=1
    urdf_str += "\t" * indent + "</geometry>\n"
    indent -=1
    urdf_str += "\t" * indent + "</collision>\n"

    urdf_str += "\t" * indent + "<contact>\n"
    indent += 1
    urdf_str += "\t" * indent + "<lateral_friction value = \""+str(friction)+"\"/>\n"
    indent -= 1
    urdf_str += "\t" * indent + "</contact>\n"


    urdf_str += "\t" * indent + "<inertial>\n"
    indent +=1
    urdf_str += "\t" * indent + "<mass value = \""+str(mass)+"\"/>\n"
    height_inertia = 5.*mass/12.
    urdf_str += "\t" * indent + "<inertia ixx=\""+str(height_inertia)+"\" ixy=\"0.0\" ixz=\"0.0\" iyy=\""+str(height_inertia)+"\" iyz=\"0.0\" izz=\""+str(mass/6.)+"\"/>\n"
    urdf_str += "\t" * indent + "<origin xyz=\"" + str(x) + " " + str(y) + " " + str(z) + "\"/>\n"
    indent -=1
    urdf_str += "\t" * indent + "</inertial>\n"

    indent -=1
    urdf_str += "\t" * indent + "</link>\n"

    if count > 0:
        urdf_str += "\t" * indent + "<joint name = \"joint" + str(count) + "\" type = \"fixed\">\n"
        indent += 1
        urdf_str += "\t" * indent + "<parent link = \"box0\"/>\n"
        urdf_str += "\t" * indent + "<child link = \"box" + str(count) + "\"/>\n"
        indent -=1
        urdf_str += "\t" * indent + "</joint>\n"

    return urdf_str, indent



def write_URDF_from_text_file(object_name, destination_folder):
    obj = file_handling.read_combined_boxes_extra_info_rigid_body_file("object models\\" + object_name + ".txt")

    urdf_str = "<robot name = \"" + object_name + "\">\n"

    indent = 1
    for count,info_list in enumerate(obj):
        x,y,z,mass,friction = info_list
        urdf_str, indent = write_URDF_basic(urdf_str, indent, count, x, y, z, mass, friction)

    urdf_str += "</robot>"

    #print(urdf_str)

    file_handling.write_urdf(destination_folder, object_name, urdf_str)


def write_URDF_with_external_mass_and_friction_info(object_name, object_data, mass_data, friction_data, destination_folder):
    urdf_str = "<robot name = \"" + object_name + "\">\n"

    indent = 1
    for count,info_list in enumerate(object_data):
        x,y,z = info_list
        mass = mass_data[count]
        friction = friction_data[count]
        urdf_str, indent = write_URDF_basic(urdf_str, indent, count, x, y, z, mass, friction)

    urdf_str += "</robot>"

    #print(urdf_str)

    file_handling.write_urdf(destination_folder, object_name, urdf_str)



def write_URDF_from_mesh(object_name, destination_folder, model_COM, extra_designation=""):
    indent = 0
    object_model_folder = os.path.join("object models",object_name)
    object_mesh_file = os.path.join("..",object_model_folder, object_name+".obj")

    urdf_str = "<robot name = \"" + object_name + "\">\n"
    indent += 1
    urdf_str += "\t" * indent + "<link name = \"main" + "\">\n"
    indent += 1

    urdf_str += "\t" * indent + "<visual>\n"
    indent += 1
    urdf_str += "\t" * indent + "<origin xyz = \"0.0 0.0 0.0\"/>\n"
    urdf_str += "\t" * indent + "<geometry>\n"
    indent += 1
    urdf_str += "\t" * indent + "<mesh filename=\""+object_mesh_file+"\"/>\n"
    indent -=1
    urdf_str += "\t" * indent + "</geometry>\n"
    urdf_str += "\t" * indent + "<material name=\""+object_name+"_material\">\n"
    indent += 1
    urdf_str += "\t" * indent + "<color rgba=\"1.0 1.0 1.0 1.0\"/>\n"
    indent -=1
    urdf_str += "\t" * indent + "</material>\n"
    indent -=1
    urdf_str += "\t" * indent + "</visual>\n"

    urdf_str += "\t" * indent + "<collision>\n"
    indent += 1
    urdf_str += "\t" * indent + "<origin xyz = \"0.0 0.0 0.0\"/>\n"
    urdf_str += "\t" * indent + "<geometry>\n"
    indent += 1
    urdf_str += "\t" * indent + "<mesh filename=\""+object_mesh_file+"\"/>\n"
    indent -=1
    urdf_str += "\t" * indent + "</geometry>\n"
    indent -=1
    urdf_str += "\t" * indent + "</collision>\n"

    urdf_str += "\t" * indent + "<contact>\n"
    indent += 1
    urdf_str += "\t" * indent + "<lateral_friction value = \"0.3\"/>\n"
    indent -=1
    urdf_str += "\t" * indent + "</contact>\n"

    urdf_str += "\t" * indent + "<inertial>\n"
    indent += 1
    urdf_str += "\t" * indent + "<origin xyz = \""+str(model_COM[0])+" "+str(model_COM[1])+" "+str(model_COM[2])+"\"/>\n"
    urdf_str += "\t" * indent + "<mass value = \"1.0\"/>\n"
    urdf_str += "\t" * indent + "<inertia ixx=\"0.166666666666667\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.16666666666666666\" iyz=\"0.0\" izz=\"0.166666666666667\"/>\n"
    indent -= 1
    urdf_str += "\t" * indent + "</inertial>\n"

    indent -=1
    urdf_str += "\t" * indent + "</link>\n"
    indent -=1
    urdf_str += "</robot>"


    file_handling.write_urdf(destination_folder, object_name+extra_designation, urdf_str)




'''
  <!-- right is either 1 (for right arm) or -1 (for left arm) -->
  <link name="calib_kuka_arm_base_link">
    <inertial>
      <mass value="0"/>
      <!-- static base, disable dynamics for this link -->
      <origin rpy="0 0 0" xyz="0 0 0.055"/>
      <inertia ixx="0.00381666666667" ixy="0" ixz="0" iyy="0.0036" iyz="0" izz="0.00381666666667"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="meshes_arm/arm_base.stl"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="meshes_arm/convex/arm_base_convex.stl"/>
      </geometry>
    </collision>
  </link>
'''