'''
Usage:
blender -b -P scan_single_around.py <model_dir> <output_dir> <model_id> <save_rgbd_image> <save_pc_per_view> <save_pc_complete> <pc_per_view_size> <pc_complete_size>

Example:
Scan the provided model.obj file using 6 fixed cameras to cover the object from all directions, output rgbd image, partial point cloud (2048 points) from each camera view,
and the complete point cloud (16384 points) by combining all camera views, the output files will be named as 'plane-xxx.xxx' and saved in './output' directory:
blender -b -P scan_single_around.py ./example_obj/model.obj ./output plane 1 1 1 2048 16384
'''

import os
import sys
import numpy as np
from mathutils import Matrix, Vector
import bpy
import bpycv
import cv2
import h5py

INTRINSIC = np.array([[575, 0.0, 320],
					  [0.0, 575, 240],
					  [0.0, 0.0, 1.0]])
FOCUS_POINT = [0.0, 0.0, 0.0]
CAMERA_DISTANCE = 1.2
LIGHT_DISTANCE = 5

def update_preset_camera(camera, focus_point, location):
	'''
	Focus the camera to focus point and locate at the specified location.
	'''
	looking_direction = focus_point - location
	rot_quat = looking_direction.to_track_quat('-Z', 'Y')
	camera.rotation_euler = rot_quat.to_euler()
	camera.location = location

def setup_blender(intrinsic, focus_point, light_distance):
	# camera intrinsic
	focal_length = intrinsic[0, 0]
	width = int(intrinsic[0, 2] * 2)
	height = int(intrinsic[1, 2] * 2)
	camera = bpy.data.objects['Camera']
	camera.data.type = 'PERSP' # or 'ORTHO'
	camera.data.angle = np.arctan(width / 2 / focal_length) * 2
	# scene setting
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
	# preset 6 fixed lights around the focus point
	light_names = ['Light_front', 'Light_back', 'Light_left', 'Light_right', 'Light_top', 'Light_bottom']
	light_locations = []
	for i in range(3):
		light_location = focus_point[:]
		light_location[i] -= light_distance
		light_locations.append(light_location)
		light_location = focus_point[:]
		light_location[i] += light_distance
		light_locations.append(light_location)
	for i in range(len(light_names)):
		light_data = bpy.data.lights.new(name=light_names[i], type='POINT')
		light_data.energy = 500
		light_object = bpy.data.objects.new(name=light_names[i], object_data=light_data)
		bpy.context.collection.objects.link(light_object)
		bpy.context.view_layer.objects.active = light_object
		light_object.location = light_locations[i]
	return camera

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
			output_dir_segid = os.path.join(output_dir, 'segid')
			os.makedirs(output_dir_color, exist_ok=True)
			os.makedirs(output_dir_depth, exist_ok=True)
			os.makedirs(output_dir_segid, exist_ok=True)
		if save_pc_per_view or save_pc_complete:
			output_dir_pc = os.path.join(output_dir, 'pc')
			os.makedirs(output_dir_pc, exist_ok=True)
		# initialize
		camera = setup_blender(INTRINSIC, FOCUS_POINT, LIGHT_DISTANCE)
		# import model that each group is splited as an object
		bpy.ops.import_scene.obj(filepath=model_dir, axis_forward='Y', axis_up='Z', use_split_groups=True)
		# deselect all objects
		bpy.ops.object.select_all(action='DESELECT')
		# assign unique id for each object
		multi_parts_flag = False
		instance_id = 0
		for obj in bpy.data.objects:
			if obj.type == "MESH":
				obj.select_set(True)
				obj["inst_id"] = instance_id
				instance_id += 1
		num_parts = instance_id + 1
		if num_parts > 1:
			multi_parts_flag = True
		# preset 6 fixed camera locations around the focus point
		cam_locations = []
		for i in range(3):
			cam_location = FOCUS_POINT[:]
			cam_location[i] -= CAMERA_DISTANCE
			cam_locations.append(cam_location)
			cam_location = FOCUS_POINT[:]
			cam_location[i] += CAMERA_DISTANCE
			cam_locations.append(cam_location)
		num_frames = len(cam_locations)
		# start scanning
		pc_complete = []
		for i in range(num_frames):
			update_preset_camera(camera, location=Vector(cam_locations[i]), focus_point=Vector(FOCUS_POINT))
			if i == 0:  # have to do this for a potential bug on Windows
				result = bpycv.render_data()
			result = bpycv.render_data()
			# save rgbd
			color_img = result["image"]
			depth_img = result["depth"]
			segid_img = result["inst"]
			if save_rgbd:
				# transfer RGB image to opencv's BGR
				cv2.imwrite(output_dir_color + '/%s-color-%d.jpg' % (model_id, i), color_img[..., ::-1])
				# convert depth units from meters to millimeters
				cv2.imwrite(output_dir_depth + '/%s-depth-%d.png' % (model_id, i), np.uint16(depth_img * 1000))
				# save instance map as 16 bit png
				cv2.imwrite(output_dir_segid + '/%s-segid-%d.png' % (model_id, i), np.uint16(segid_img))
			# save point cloud
			if save_pc_per_view or save_pc_complete:
				pc_per_view = []
				if multi_parts_flag:
					for k in range(num_parts):
						depth_img_part = np.where(segid_img == k, depth_img, 0)
						part_pcd = depth2pcd(depth_img_part, INTRINSIC, np.array(camera.matrix_world), color_img)
						part_index = np.repeat(k, len(part_pcd))
						part_pcd = np.column_stack((part_pcd, part_index))
						if k == 0:
							pc_per_view = part_pcd
						else:
							pc_per_view = np.concatenate((pc_per_view, part_pcd), axis=0)
				else:
					pc_per_view = depth2pcd(depth_img, INTRINSIC, np.array(camera.matrix_world), color_img)
				if save_pc_complete:
					if i == 0:
						pc_complete = pc_per_view
					else:
						pc_complete = np.concatenate((pc_complete, pc_per_view), axis=0)
				if save_pc_per_view:
					if pc_per_view.shape[0] >= pc_per_view_size:
						np.random.shuffle(pc_per_view)
						pc_per_view = pc_per_view[:pc_per_view_size, :]
						''' save as .h5 '''
						# with h5py.File(os.path.join(output_dir_pc + '/%s-perview-%d.h5' %(model_id, i)), 'w') as f:
						#     f.create_dataset(name="data", data=np.array(pc_per_view).astype(np.float32), compression="gzip")
						''' save as .pts '''
						np.savetxt(os.path.join(output_dir_pc + '/%s-perview-%d.pts' % (model_id, i)), pc_per_view, fmt='%.8f')
					else:
						print('Points number in frame %d is %d, fewer than %d' % (i, pc_per_view.shape[0], pc_per_view_size))
						with open(os.path.join(output_dir, 'pcsize_error.txt'), 'a') as f_exp:
							f_exp.writelines(model_id+'_frame_'+str(i)+'\n')
		if save_pc_complete:
			np.random.shuffle(pc_complete)
			if pc_complete.shape[0] >= pc_complete_size:
				pc_complete = pc_complete[:pc_complete_size, :]
				''' save as .h5 '''
				# with h5py.File(os.path.join(output_dir_pc + '/%s-complete.h5' % model_id), 'w') as f:
				#         f.create_dataset(name="data", data=np.array(pc_complete).astype(np.float32), compression="gzip")
				''' save as .pts '''
				np.savetxt(os.path.join(output_dir_pc + '/%s-complete.pts' % model_id), pc_complete, fmt='%.8f')
			else:
				print('Points number is %d, fewer than %d' % (pc_complete.shape[0], pc_complete_size))
				with open(os.path.join(output_dir, 'pcsize_error.txt'), 'a') as f_exp:
					f_exp.writelines(model_id+'_complete\n')
		# clean up objects
		bpy.ops.object.delete()
		os.close(1)
	else:
		print('Load file error')
		with open(os.path.join(output_dir, 'load_error.txt'), 'a') as f_exp:
			f_exp.writelines(model_id+'\n')
