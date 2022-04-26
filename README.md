## 详细解释来自于博客https://blog.csdn.net/jiny_yang/article/details/115590451
#### 使用YOLOv5对图像做检测，把检测框保留在result文件夹内；
#### 使用orbslam2读取文件夹内的检测框，并剔除动态特征点

#### 编译方法
参考ORBSLAM2

### 运行方法
#### 对于 TUM/rgbd_dataset_freiburg3_walking_xyz 数据集：

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml path/to/rgbd_dataset_freiburg3_walking_xyz path/to/associate.txt detect_result/TUM_f3xyz_yolov5m/detect_result/

##### 我的运行方法：

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml ~/Desktop/dataset/TUM/rgbd_dataset_freiburg3_walking_xyz ~/Desktop/dataset/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt detect_result/TUM_f3xyz_yolov5m/detect_result/

#### 对于rgbd_dataset_freiburg3_walking_halfsphere数据集：

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml path/to/rgbd_dataset_freiburg3_walking_halfsphere path/to/associate.txt detect_result/TUM_f3halfsphere_yolov5x/detect_result/

##### 我的运行方法：

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml ~/Desktop/dataset/TUM/rgbd_dataset_freiburg3_walking_halfsphere/ ~/Desktop/dataset/TUM/rgbd_dataset_freiburg3_walking_halfsphere/associate.txt detect_result/TUM_f3halfsphere_yolov5x/detect_result/



