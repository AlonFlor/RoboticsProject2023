import os
import numpy as np
#import geometry_utils

#file handling

def read_file(file):
    data_raw = []
    for line in file:
        data_raw.append(line.strip().split(","))
    data = []
    for line in data_raw[1:]:
        line_data = []
        for item in line:
            line_data.append(float(item))
        data.append(line_data)
    return np.array(data)


def read_robot_lab_data(robot_lab_data_folder):
    pose_data_file = open(os.path.join(robot_lab_data_folder, "outobjectposes.csv"))
    forces_data_file = open(os.path.join(robot_lab_data_folder, "outforce.csv"))
    end_effector_data_file = open(os.path.join(robot_lab_data_folder, "outendeff.csv"))

    pose_data = read_file(pose_data_file)[0]
    forces_data = read_file(forces_data_file)
    end_effector_data = read_file(end_effector_data_file)

    pose_data_file.close()
    forces_data_file.close()
    end_effector_data_file.close()

    return pose_data, forces_data, end_effector_data



def write_records_and_motion_script(shapes_list, dir_name, motion_script):

    #write file with locations of shapes
    shapes_list_file_loc = os.path.join(dir_name, "shapes.csv")
    shapes_list_file = open(shapes_list_file_loc, "w")
    for shape,model_com in shapes_list:
        #write shape name and COM
        shapes_list_file.write(shape + "," + str(model_com[0]) + "," + str(model_com[1]) + "," + str(model_com[2]) + "\n")
    shapes_list_file.close()

    #write file with shapes' motions
    motion_script_loc = os.path.join(dir_name, "motion_script.csv")
    motion_script_file = open(motion_script_loc, "w")
    motion_script_file.write("shape,time,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w\n")

    for i in np.arange(len(motion_script)):
        shape_motion_data = motion_script[i]
        for j in np.arange(len(shape_motion_data)):
            time_val,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w = shape_motion_data[j]
            motion_script_file.write(str(shapes_list[i][0])+",")
            motion_script_file.write(str(time_val)+",")
            motion_script_file.write(str(x)+",")
            motion_script_file.write(str(y)+",")
            motion_script_file.write(str(z)+",")
            motion_script_file.write(str(orientation_x)+",")
            motion_script_file.write(str(orientation_y)+",")
            motion_script_file.write(str(orientation_z)+",")
            motion_script_file.write(str(orientation_w)+"\n")

    motion_script_file.close()









def read_combined_boxes_rigid_body_file(file_loc):
    file = open(file_loc, "r", encoding="utf-8")
    stuff = file.readlines()
    file.close()

    info = []
    for line in stuff:
        line_stripped = line.strip()
        x,y,z = line_stripped.split(",")
        info.append((float(x), float(y), float(z)))
    return info

def read_combined_boxes_extra_info_rigid_body_file(file_loc):
    file = open(file_loc, "r", encoding="utf-8")
    stuff = file.readlines()
    file.close()

    info = []
    for line in stuff[1:]:
        line_stripped = line.strip()
        x,y,z,mass,friction = line_stripped.split(",")
        info.append((float(x), float(y), float(z), float(mass), float(friction)))
    return info

def write_urdf(object_loc, object_name, urdf_str):
    file = open(os.path.join(object_loc, object_name) +".urdf", "w", encoding="utf-8")
    file.write(urdf_str)
    file.close()

def write_obj_and_mtl_one_frame(shapes_loc_data, masses_data, frictions_data, com_actual_loc, com_found_loc, shape_name, dir_name, mode):

    #assume for now that max mass is 20 and max friction is 1.
    max_mass = 20.
    max_friction = 1.

    #write material file
    material = open(os.path.join(dir_name, "materials_"+mode+".mtl"), "w")
    material.write("newmtl com_actual\n") # material for actual center of mass
    material.write("Ka 1. 1. 1.\n")
    material.write("Kd 1. 1. 1.\n")
    material.write("Ks 0.0 0.0 0.0\n")
    material.write("newmtl com_found\n") # material for found center of mass
    material.write("Ka 0. 0. 0.\n")
    material.write("Kd 0. 0. 0.\n")
    material.write("Ks 0.0 0.0 0.0\n")
    if mode == "mass":
        for shape_count in range(len(shapes_loc_data)):
            # mass material, green = maximum mass, red = massless
            material.write("newmtl " + "{:0>4d}".format(shape_count) + "\n")
            amount = masses_data[shape_count] / max_mass
            material.write("Ka " + str(1.-amount) + " " + str(amount) + " " + " 0.\n")
            material.write("Kd " + str(1.-amount) + " " + str(amount) + " " + " 0.\n")
            material.write("Ks 0.0 0.0 0.0\n")
    elif mode == "friction":
        for shape_count in range(len(shapes_loc_data)):
            # friction material, green = maximum friction, red = frictionless
            material.write("newmtl " + "{:0>4d}".format(shape_count) + "\n")
            amount = frictions_data[shape_count] / max_friction
            material.write("Ka " + str(1.-amount) + " " + str(amount) + " " + " 0.\n")
            material.write("Kd " + str(1.-amount) + " " + str(amount) + " " + " 0.\n")
            material.write("Ks 0.0 0.0 0.0\n")
    elif mode == "mass_times_friction":
        for shape_count in range(len(shapes_loc_data)):
            # mass*friction material, green = maximum mass*friction, red = massless and/or frictionless
            material.write("newmtl " + "{:0>4d}".format(shape_count) + "\n")
            amount = masses_data[shape_count] * frictions_data[shape_count] / (max_mass * max_friction)
            material.write("Ka " + str(1.-amount) + " " + str(amount) + " " + " 0.\n")
            material.write("Kd " + str(1.-amount) + " " + str(amount) + " " + " 0.\n")
            material.write("Ks 0.0 0.0 0.0\n")
    material.close()

    # write a frame for blender
    out_file = open(os.path.join(dir_name, shape_name + "_" + mode + ".obj"), "w")
    out_file.write("#" + shape_name + "\n")
    out_file.write("mtllib materials_" + mode + ".mtl\n")  # materials file to be generated by blender

    #write vertices for actual and found center of mass
    point_vertices = [np.array([-0.1, 0., 1.01]), np.array([0., -0.1, 1.01]), np.array([0.1, 0., 1.01]), np.array([0., 0.1, 1.01])]
    for vertex in point_vertices:
        world_vertex = vertex + com_actual_loc
        out_file.write("v")
        for coord in world_vertex:
            out_file.write(" " + str(coord))
        out_file.write("\n")
    for vertex in point_vertices:
        world_vertex = vertex + com_found_loc
        out_file.write("v")
        for coord in world_vertex:
            out_file.write(" " + str(coord))
        out_file.write("\n")

    # write vertices for component shapes
    vertices_index_shift = [8]
    vertices = [np.array([-0.5, -0.5, 0.]), np.array([-0.5, 0.5, 0.]), np.array([0.5, -0.5, 0.]), np.array([0.5, 0.5, 0.]),
                np.array([-0.5, -0.5, 1.]), np.array([-0.5, 0.5, 1.]), np.array([0.5, -0.5, 1.]), np.array([0.5, 0.5, 1.])]
    for shape_loc_data in shapes_loc_data:
        loc = np.array(shape_loc_data)
        vertices_added = 0
        for vertex in vertices:
            world_vertex = vertex + loc
            out_file.write("v")
            for coord in world_vertex:
                out_file.write(" " + str(coord))
            out_file.write("\n")
            vertices_added += 1
        vertices_index_shift.append(vertices_added)

    # update vertex indicies for writing faces
    for i in np.arange(1, len(vertices_index_shift)):
        vertices_index_shift[i] += vertices_index_shift[i - 1]

    # write faces for actual and found centers of mass
    out_file.write("usemtl com_actual\n")
    out_file.write("f")
    for i in np.arange(4):
        out_file.write(" " + str(i + 1))
    out_file.write("\n")
    out_file.write("usemtl com_found\n")
    out_file.write("f")
    for i in np.arange(4):
        out_file.write(" " + str(i + 5))
    out_file.write("\n")

    # write faces for component shapes
    faces = [([0, 1, 3, 2], 0),
             ([4, 5, 7, 6], 1),
             ([0, 1, 5, 4], 2),
             ([2, 3, 7, 6], 3),
             ([0, 2, 6, 4], 4),
             ([1, 3, 7, 5], 5)]
    for shape_count in np.arange(len(shapes_loc_data)):
        out_file.write("usemtl " + "{:0>4d}".format(shape_count) + "\n")

        for vertex_indices, normal_index in faces:
            out_file.write("f")
            for vertex_index in vertex_indices:
                out_file.write(" " + str(vertex_index + vertices_index_shift[shape_count] + 1))
            out_file.write("\n")
