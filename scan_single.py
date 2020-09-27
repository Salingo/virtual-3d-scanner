import os
import sys
import bpy
import bpycv
import cv2
import h5py
import numpy as np
from mathutils import Matrix

intrinsic = np.array([[575, 0.0, 320],
                      [0.0, 575, 240],
                      [0.0, 0.0, 1.0]])

def custom_camera_extrinsic():
    # here preset 6 fixed camera around the origin
    custom_extrinsic = []
    dist_to_origin = 1.5
    rot_axis_list = ['Y', 'X', 'X']
    rot_angle_list = [[np.pi / 2.0, -np.pi / 2.0], [-np.pi / 2.0, np.pi / 2.0], [0, np.pi]]
    for i in range(3):
        for j in range(2):
            dist_factor = 1 if j == 0 else -1
            translation = [0, 0, 0]
            translation[i] = dist_to_origin * dist_factor
            translation = Matrix.Translation(translation)            
            rotation = Matrix.Rotation(rot_angle_list[i][j], 4, rot_axis_list[i])
            extrinsic = Matrix(np.matmul(translation, rotation))
            custom_extrinsic.append(extrinsic)
    return custom_extrinsic

def setup_blender(width, height, focal_length):
    # camera
    camera = bpy.data.objects['Camera']
    camera.data.type = 'PERSP' # or 'ORTHO'
    camera.data.angle = np.arctan(width / 2 / focal_length) * 2

    # render layer
    scene = bpy.context.scene
    scene.render.film_transparent = True  # set transparent background
    scene.render.image_settings.color_depth = '16'
    scene.render.image_settings.use_zbuffer = True
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100

    # remove default cube and light
    bpy.data.objects['Cube'].select_set(state=True)
    bpy.ops.object.delete()
    bpy.data.objects['Light'].select_set(state=True)
    bpy.ops.object.delete()

    # create 6 surrounding lights
    lights = ['light_front', 'light_back', 'light_left', 'light_right', 'light_top', 'light_bottom']
    lights_poses = np.array([[0,0,5],[0,0,-5],[0,5,0],[0,-5,0],[5,0,0],[-5,0,0]])
    num_lights=len(lights)
    for i in range(0, num_lights):
        light_data = bpy.data.lights.new(name=lights[i], type='POINT')
        light_data.energy = 500
        light_object = bpy.data.objects.new(name=lights[i], object_data=light_data)
        bpy.context.collection.objects.link(light_object)
        bpy.context.view_layer.objects.active = light_object
        light_object.location = lights_poses[i]

    # compositor nodes
    scene.use_nodes = True
    tree = scene.node_tree
    for n in tree.nodes:
        tree.nodes.remove(n)
    tree.nodes.new('CompositorNodeRLayers')

    return scene, camera

def depth2pcd(depth, intrinsic, pose=None, colors=None):
    # camera coordinate system in Blender is x: right, y: up, z: inwards
    inv_K = np.linalg.inv(intrinsic)
    inv_K[2, 2] = -1
    depth = np.flipud(depth)
    y, x = np.where(depth > 0)
    points = np.dot(inv_K, np.stack([x, y, np.ones_like(x)] * depth[y, x], 0))
    if pose is not None:
        points = np.dot(pose, np.concatenate([points, np.ones((1, points.shape[1]))], 0))[:3, :]
    points = points.T
    if colors is not None:
        colors = np.flipud(colors) / 255.0
        points = np.concatenate([points, colors[y, x, :3]], axis=1)
    return points


if __name__ == '__main__':
    print(sys.argv)
    model_dir = sys.argv[-8]
    output_dir = sys.argv[-7]
    model_id = sys.argv[-6]
    save_rgbd = int(sys.argv[-5])
    save_pc_per_view = int(sys.argv[-4])
    save_pc_complete = int(sys.argv[-3])
    pc_per_view_size = int(sys.argv[-2])
    pc_complete_size = int(sys.argv[-1])

    if os.path.isfile(model_dir):
        # output directory settings
        if save_rgbd:
            output_dir_color = os.path.join(output_dir, 'color')
            output_dir_depth = os.path.join(output_dir, 'depth')
            os.makedirs(output_dir_color, exist_ok=True)
            os.makedirs(output_dir_depth, exist_ok=True)
        if save_pc_per_view or save_pc_complete:
            output_dir_pc = os.path.join(output_dir, 'pc')
            os.makedirs(output_dir_pc, exist_ok=True)

        # camera settings
        focal = intrinsic[0, 0]
        width = int(intrinsic[0, 2] * 2)
        height = int(intrinsic[1, 2] * 2)
        extrinsic = custom_camera_extrinsic()
        scene, camera = setup_blender(width, height, focal)

        # import model
        bpy.ops.import_scene.obj(filepath=model_dir, axis_forward='Y', axis_up='Z')

        # start scanning
        pc_complete = []
        num_frames = len(extrinsic)
        for i in range(num_frames):
            camera.matrix_world = extrinsic[i]
            scene.frame_set(i)

            if i == 0:  # have to do this for a potential bug
                result = bpycv.render_data()
            result = bpycv.render_data()

            # save rgbd
            color_img = result["image"]
            depth_img = result["depth"]
            if save_rgbd:
                cv2.imwrite(output_dir_color + '/%s-color-%d.jpg' % (model_id, i), color_img[..., ::-1])  # transfer RGB image to opencv's BGR
                cv2.imwrite(output_dir_depth + '/%s-depth-%d.png' % (model_id, i), np.uint16(depth_img * 1000))  # convert depth units from meters to millimeters

            # save point cloud
            if save_pc_per_view or save_pc_complete:
                pc_per_view = depth2pcd(depth_img, intrinsic, np.array(extrinsic[i]), color_img)

                if save_pc_complete:
                    if i == 0:
                        pc_complete = pc_per_view
                    else:
                        pc_complete = np.concatenate((pc_complete, pc_per_view), axis=0)

                if save_pc_per_view:
                    if pc_per_view.shape[0] >= pc_per_view_size:
                        np.random.shuffle(pc_per_view)
                        pc_per_view = pc_per_view[:pc_per_view_size, :]
                        pc_per_view = pc_per_view + np.random.rand(pc_per_view.shape[0], pc_per_view.shape[1]) * 0.005  # add noise to simulate real scanner
                        # Save as .h5
                        with h5py.File(os.path.join(output_dir_pc + '/%s-perview-%d.h5' %(model_id, i)), 'w') as f:
                            f.create_dataset(name="data", data=np.array(pc_per_view).astype(np.float32), compression="gzip")
                        # Save as .pts
                        # np.savetxt(os.path.join(output_dir_pc + '/%s-per_view-%d.pts' % (model_id, i)), pc_per_view, fmt='%.8f')
                    else:
                        print('Points number is %d, fewer than %d' % (pc_per_view.shape[0], pc_per_view_size))
                        with open(os.path.join(output_dir, 'pcsize_error.txt'), 'a') as f_exp:
                            f_exp.writelines(model_id+'\n')

        if save_pc_complete:
            np.random.shuffle(pc_complete)
            if pc_complete.shape[0] >= pc_complete_size:
                pc_complete = pc_complete[:pc_complete_size, :]
                pc_complete = pc_complete + np.random.rand(pc_complete.shape[0], pc_complete.shape[1]) * 0.005  # add noise to simulate real scanner
                # Save as .h5
                with h5py.File(os.path.join(output_dir_pc + '/%s-complete.h5' % model_id), 'w') as f:
                        f.create_dataset(name="data", data=np.array(pc_complete).astype(np.float32), compression="gzip")
                # Save as .pts
                # np.savetxt(os.path.join(output_dir_pc + '/%s-complete.pts' % model_id), pc_complete, fmt='%.8f')
            else:
                print('Points number is %d, fewer than %d' % (pc_complete.shape[0], pc_complete_size))
                with open(os.path.join(output_dir, 'pcsize_error.txt'), 'a') as f_exp:
                    f_exp.writelines(model_id+'\n')

        # Clean up object
        bpy.ops.object.delete()
        os.close(1)

    else:
        print('Load file error')
        with open(os.path.join(output_dir, 'load_error.txt'), 'a') as f_exp:
            f_exp.writelines(model_id+'\n')
