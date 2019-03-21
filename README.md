This is part of my [Winter Project at NWU](https://github.com/felixchenfy/Detect-Object-and-6D-Pose).

# **Function:**
Use a depth camera to detect objects on table and get their mask in 2D image.

<p align = "center"> 
  <img src = "doc/RealsenseDepthCamera.png" height = "40px">
</p>
<p align = "center"> 
  <img src = "doc/picture_demo.png" height = "300px">
</p>

# **Algorithm**
1. Get the point cloud from rgbd image (rgbd image = color + depth image)  
2. Detect and remove plane (table surface). The rest clouds are the objects.   
3. Do clustering. Each cluster is an object.  
4. Project 3D points onto 2D image to obtain the mask of objects.  
5. Do a dilation and erosion to fill holes, and get the final mask.  


# **How to run**

## **1. Settings**

### 1.1 Set data source in [launch/main.launch](launch/main.launch)

There are 3 choices for the input RGBD-D images:  
(1). Realsense camera.  
(2). AsusXtion camera.  
(3). Load from file.

### 1.2 Set filtering in [src/n1_main_add_label_to_image.py](src/n1_main_add_label_to_image.py)

Search for "ObjectDetectorFromRGBD". There are several parameters such as **"zmin"**, **"zmax"**, etc., which are for **range filtering** of the input cloud, and **"voxel_size"** for downsampling the input cloud.

### 1.3 Set object detection in [launch/main.launch](launch/main.launch)

Search for "node_detect_object_from_cloud". There are several paramters related to **removing plane** and **do clustering**. You can modify them.


## **2. Open a depth camera**

Based on the data source you choosed in step 1, run one of the follows:  

> $ roslaunch mask_objects_from_rgbd open_realsense.launch  

or:

> $ roslaunch mask_objects_from_rgbd open_xtion.launch  

or:

> Do nothing if you choose to load from file. But remember to modify the paths of data source in [src/n0_fake_rgbd_image_publisher.py](src/n0_fake_rgbd_image_publisher.py).

## **3. Launch!**
> $ roslaunch mask_objects_from_rgbd main.launch  

## **4. Check and save result**

The result is shown in rviz, similar to the demo image above.

You can also press '**s**' to save the results. They will be saved into [data/](data) folder as follows:
```
data
└── 03-21-14-03-22-686
    ├── clouds
    ├── depth
    ├── image
    ├── mask
    ├── objects
    └── resimg
```
(Notes: I'm using cv2.waitKey(), so you need to click the showed image, and then press 's'.)