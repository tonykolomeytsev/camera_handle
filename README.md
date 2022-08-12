# The `camera_handle` ROS node

A node that switches between different camera topics and publishes images of the selected camera to a single topic.

<p align="center">
  <img src="https://github.com/tonykolomeytsev/camera_handle/raw/master/media/diagram.png" alt="diagram"/>
</p>

## How it works

The `camera_handle` node subscribes to `/camera/handle` topic which has `std_msgs/String` type. The node also publish frames from selected topic to its output topic (`/camera/image_raw` by default). You can select new topic by name, publishing the name to `/camera/handle` topic. Every time the node receives new name, it unsubscribes from previous topic and subscribes to the new.

You can manually test the node by publishing some name to topic `/camera/handle` via `rostopic pub`:

```bash
rostopic pub /camera/handle std_msgs/String '/cam2/image_raw'
```

## Usage

### Using prebuilt docker image

Just run new container from `tonykolomeytsev/camera_handle:latest`:

```bash
docker run --net host --privileged --rm -it tonykolomeytsev/camera_handle:latest
```

And start `roslaunch` inside it:

```bash
roslaunch camera_handle handle.launch
```

### Building your own docker image

Clone the repo and run `docker build` with the provided [Dockerfile](https://github.com/tonykolomeytsev/camera_handle/blob/master/docker/Dockerfile).

```bash
git clone https://github.com/tonykolomeytsev/camera_handle.git
docker build -t camera_handle ./camera_handle/docker/
```

And then run built image in new container:

```bash
docker run --net host --privileged --rm -it camera_handle
```

And start `roslaunch` inside it:

```bash
roslaunch camera_handle handle.launch
```

### Building as catkin package

> ROS Noetic required for this purpose

Clone the repo to the catkin workspace, and run `catkin_make`.

```bash
cd <catkin_workspace>
git clone https://github.com/tonykolomeytsev/camera_handle.git src/camera_handle
catkin_make camera_handle
source devel/setup.bash
```

And then a standard procedure to run:
```bash
roslaunch camera_handle handle.launch
```

## Launch file parameters

- `output_topic` is the name of the topic where you want to publish frames of the selected camera. Default is `/camera/image_raw`.
    ```xml
    <param name="output_topic" type="string" value="/camera/image_raw" />
    ```
- `initial_topic` is the name of the input topic that will be selected when the node starts (so as not to publish this name in the `/camera/handle` topic). Empty by default - the node will not subscribe to any topic until the name comes in /camera/handle of the topic.
    ```xml
    <param name="initial_topic" type="string" value="/cam0/image_raw" />
    ```
