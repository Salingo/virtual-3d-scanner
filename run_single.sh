#!/bin/bash

input_dir=${D:\models\model.obj}
output_dir=${D:\render}
model_id=${chair_01}
save_rgbd_image=${1}
save_pc_perframe=${1}
save_pc_complete=${1}
pc_perframe_size=${4096}
pc_complete_size=${16384}

blender -b -P scan_single.py $input_dir $output_dir $model_id $save_rgbd_image $save_pc_perframe $save_pc_complete $pc_perframe_size $pc_complete_size
