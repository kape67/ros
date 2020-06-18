# ros

```shell
$ rosrun save_img save_img rgb_img:=[rgb topic name]
```

If you use kinect2, then

```shell
$ rosrun save_img save_img rgb_img:=/kinect2/qhd/image_color
```

If you use realsense camera, then

```shell
$ rosrun save_img save_img rgb_img:=/camera/color/image_raw
```
