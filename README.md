函数及接口说明：

 LoadClouds(cloud);//读取点云

PlaneSet_S.Extrace_plane(）;//提取平面
CylinderSet_S.Extrace_cylinder();//提取圆柱
Viewer_Results_clouds();//可视化点云



PlaneSet_S里存放类型为<plane>的平面集合，plane有cofficient_set（模型系数）和cloud_plane（分割出来的点云）

CylinderSet_S类似

平面个数、圆柱的最少聚类点（少于这个值不认为是一个圆柱）、平面的内点阈值、圆柱的内点阈值、下采样的体素网格大小均需要自己调参，参数均在main.cpp的int main()方法里



调参：

1.scan1.pcd

下采样DownSampleLeafSize：20.0f, 20.0f, 20.0f（下采样后1w）

平面个数MAX_ITERATION_PLANE：1（需自己手动控制）

平面阈值PlaneThresold：40

圆柱最少聚类点个数minPointNum：600

圆柱阈值CylinderThresold：14（13-16均可 14效果最好）

不去离群点（If_OutFilter==false）若加上去离群点需要重新调参

![](E:\v5\lab\图片\scan1.png)

2.acmh_model.pcd

下采样：0.009f, 0.009f, 0.009f（下采样后4w）

平面个数：1

平面阈值：0.035

圆柱最少聚类点个数：4000

圆柱阈值：0.07

不去离群点（If_OutFilter==false）

![](E:\v5\lab\图片\acmh1.png)

3.acmm_model.pcd

需要用到去离群点的函数（下采样后5w）

下采样DownSampleLeafSize=0.02f（0.009f噪点少了很多 但是数据量有28w）

是否需要去离群点： If_OutFilter =true

平面个数 MAX_ITERATION_PLANE：1

平面阈值PlaneThresold：0.05

圆柱最少聚类点个数 minPointNum：3500

圆柱阈值CylinderThresold：0.02



![1690655339760](C:\Users\xia\AppData\Roaming\Typora\typora-user-images\1690655339760.png)

![1690706390168](C:\Users\xia\AppData\Roaming\Typora\typora-user-images\1690706390168.png)