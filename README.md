# 1、计算点云重叠率：

重叠率计算：

src tgt 为统一坐标系且配准完成的点云

tgt影射到kdtree中，通过对src中点进行间隔性遍历，在tgt中查找对应点，通过距离与阈值进行判断是否为重叠点。最终

重叠率=重叠点个数/（（总数 /间隔点数））

### step1：

更改cal_overlap_ratio.launch 中的文件路径为对应本地的路径（点云文件位于src/data文件中）

```xml
<param name="src_path" type="string" value="/home/czy/czy_part-time_job/ws_319/src/cal_overlap_ratio/data/src_full.pcd"/>
<param name="tgt_path" type="string" value="/home/czy/czy_part-time_job/ws_319/src/cal_overlap_ratio/data/tgt_full.pcd"/>
<param name="has_Rt" type="bool" value="False"/>
```

如果没有真实RT 则默认通过ICP进行配准。

### step2：

如果存在RT则更改路径

```xml
<param name="Rt_path" type="string" value="*.txt"/>
```

txt中变换矩阵为1 * 12 形式。

测试1：src_full和tgt_full

两帧未配准点云 通过ICP配准，计算重叠率约为0.93 。因为进行voxel下采样 数据较为准确。

测试2：src_full 和src_0.8

src_0.8为src点云进行局部采样获得，采样率为0.80，通过与src进行重叠率计算，结果约为0.82，同样经过下采样 ，存在误差。

src_0.8的产生方式如generate_pverlap.launch对应，见标题2对应。

# 2、产生部分重叠点云：

更改路径为本地对应路径，其中ratio代表重叠率。

```xml
<node name="cal_overlap_ratio" pkg="cal_overlap_ratio" type="gen_overlap_clouds" output="screen">
    <param name="pcd_path" type="string" value="/home/czy/czy_part-time_job/ws_319/src/cal_overlap_ratio/data/src_full.pcd"/>
    <param name="save_path" type="string" value="/home/czy/czy_part-time_job/ws_319/src/cal_overlap_ratio/data/src_0.80.pcd"/>
    <param name="ratio" type="double" value="0.80" />
</node>
```



