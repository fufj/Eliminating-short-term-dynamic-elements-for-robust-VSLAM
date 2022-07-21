# Eliminating-Short-term-Dynamic-Elements

Eliminating-Short-term-Dynamic-Elements for robust visual SLAM.

# Building Eliminating-Short-term-Dynamic-Elements
- Install ORB-SLAM2 prerequisites: C++11 or C++0x Compiler, Pangolin, OpenCV and Eigen3  (https://github.com/raulmur/ORB_SLAM2).
- Clone the repository:
```
git clone https://github.com/fufj/Eliminating-short-term-dynamic-elements-for-robust-VSLAM
```
- Execute:
```
Note: You should download an extra weights file of the object detection module before the following execution.

cd Eliminating-Short-term-Dynamic-Elements
chmod +x build.sh
./build.sh
```

We have tested the library in **Ubuntu 16.04**, with OpenCV 3.4.

# RGB-D Example (TUM Dataset)

- Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

- Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools):

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```
- Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER` to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./rgbd Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# Acknowledgements
Our code builds on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).
