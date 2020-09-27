import os
import sys
import subprocess
from functools import partial
from multiprocessing.dummy import Pool

blender_command = 'blender'

if __name__ == '__main__':
    num_process = int(sys.argv[-8])
    dataset_dir = sys.argv[-7]
    output_dir = sys.argv[-6]
    save_rgbd_image = sys.argv[-5]
    save_pc_per_view = sys.argv[-4]
    save_pc_complete = sys.argv[-3]
    pc_per_view_size = sys.argv[-2]
    pc_complete_size = sys.argv[-1]

    category_id_list = [i for i in os.listdir(dataset_dir) if os.path.isdir(os.path.join(dataset_dir, i))]
    category_dir_list = [os.path.join(dataset_dir, i) for i in category_id_list]

    model_id_list = []
    model_dir_list = []
    commands = []
    for category, category_dir in zip(category_id_list, category_dir_list):
        model_id_list = [i for i in os.listdir(category_dir) if os.path.isdir(os.path.join(category_dir, i))]
        model_dir_list = [os.path.join(category_dir, i, 'models', 'model_normalized.obj') for i in model_id_list]

        commands.extend([[blender_command, '-b', '-P', 'scan_single.py', model_dir, os.path.join(output_dir, category), model_id, save_rgbd_image, save_pc_per_view, save_pc_complete, pc_per_view_size, pc_complete_size] for model_dir, model_id in zip(model_dir_list, model_id_list)])

    pool = Pool(num_process)
    print('=== Rendering %d models on %d workers ===' % (len(commands), num_process))
    for idx, completed in enumerate(pool.imap(partial(subprocess.run), commands)):
        print('Finished (%d / %d)' % (idx + 1, len(commands)))
