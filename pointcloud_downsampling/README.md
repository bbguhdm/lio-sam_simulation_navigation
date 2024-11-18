# Pointcloud Downsampling

ROS2 Humble implementation of Pointcloud2 Voxel Filtering using PCL.

## Preparation

- OS requirements

    Tested in Ubuntu 22.04 for ROS2 Humble;

- Dependencies

    Install ROS2 and library dependencies with:

    ```shell
    sudo apt-get install libeigen3-dev
    sudo apt install ros-humble-pcl-ros
    sudo apt install ros-humble-pcl-conversions
    ```

## Build & Run

- Build with:

    ```shell
    colcon build --symlink-install
    ```

- Run with:

    ```shell
    source install/setup.bash
    ros2 launch pointcloud_downsampling pointcloud_downsampling.launch.py
    ```

## Configuration

This ROS 2 node provides options for configuring its behavior through parameters. The configuration parameters can be set in the `config/pointcloud_downsampling.yaml` file.

- **`sub_topic`** (string, default: "/cloud_registered_body"): The topic to subscribe to for input point cloud data.

- **`pub_topic`** (string, default: "/cloud_registered_body_downsampling"): The topic to publish the downsampled point cloud data.

- **`pub_cloud_frame`** (string, default: "livox_frame"): The frame ID for the published point cloud.

- **`leaf_size_x,y,z`** (float, default: 0.2, 0.2, 0.2): The voxel grid size to 1x1x1cm. The larger the voxel grid, the more pronounced the downsampling effect.

- **`downsample_all_data`** (boolean, default: false): Enable/disable downsampling of all data.

### Example Configuration File

```yaml
sub_topic: "/cloud_registered_body"
pub_topic: "/cloud_registered_body_downsampling"
pub_cloud_frame: "livox_frame"
leaf_size_x: 0.2
leaf_size_y: 0.2
leaf_size_z: 0.2
downsample_all_data: false
```