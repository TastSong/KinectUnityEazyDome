KinectUnityEazyDome
===================================

总述
--------------------------------------

* 此程序实现了Kinect2与unity的简单结合
* 主要以官方的DOME为演示

实现功能
-------------------------------------

* 玩家与模型人物的动作同步
* 球跟随用户的右手移动
* 挥手切换图片
* 用手来拖拽场景中的物体

程序实现步骤
------------------------------------

* 安装[Kinect2驱动](https://www.microsoft.com/en-us/download/details.aspx?id=44561)
* unity新建3D项目
* 导入[KinectForWindows_UnityPro](https://github.com/TastSong/KinectUnityEazyDome/tree/master/MyKinectUnityTest/KinectForWindows_UnityPro_2.0.1410)包中的`Kinect.2.0.1410.19000`
* 再导入[Kinect v2 with MS-SDK20](https://github.com/TastSong/KinectUnityEazyDome/tree/master/MyKinectUnityTest/Kinect%20v2%20with%20MS-SDK20)中的`Kinect v2 with MS-SDK20`
* 上述四个DOME的场景分别在`Assets`文件夹下的`AvatarsDemo`、`GesturesDemo`、`InteractionDemo`和`OverlayDemo`

#### 注：项目我已经通过上述方法运行过，你可以直接用unity打开项目，也可以将两个unity包复制在自己的新项目中进行重建。
