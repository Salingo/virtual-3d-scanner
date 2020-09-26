# Virtual 3D Scanner

## Introduction

This script is designed to simulate real world 3D scanners (Kinect, PrimeSense, etc.) in virtual environments. With this tool, one can easily create RGB-D or point cloud dataset from synthetic models, such dataset could be useful in the field related to computer graphics and computer vision.

Given a 3D model, a set of customizable virtual cameras scan the model from different views, the RGB-D images and corresponding point clouds are then produced.

The script is running on [Blender](https://www.blender.org), the render core is from [bpycv](https://github.com/DIYer22/bpycv).


## Installation

- Blender (tested on version 2.8x, 2.9)
    - Setup the python environment of Blender
    ```
    cd <path to blender>\2.90\python\bin
    python -m ensurepip
    python -m pip install --upgrade pip setuptools wheel
    python -m pip install opencv-python openexr bpycv h5py
    ```
- bpycv and other dependencies
    ```
    python -m pip install opencv-python openexr bpycv
    ```
- h5py (optional)
    - Save point cloud compactly. Save 70% space comparing to pts format.
    ```
    python -m pip install h5py
    ```


## Usage

Custom the arguments in `run_single.sh` and run it !

- Custom intrinsic :
  
    Modify the intrinsic matrix in `scan_single.py`

- Custom virtual cameras :

    Modify the `custom_camera_extrinsic` function in `scan_single.py`
