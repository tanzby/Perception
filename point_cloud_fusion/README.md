# point_cloud_fusion



**参数**

| 名字              |    类别    |             说明             |
| ----------------- | :--------: | :--------------------------: |
| `input_topics`    | 字符串数组 | 被用于合并的点云话题名字数组 |
| `output_topic`    |   字符串   |   合并后输出点云的话题名字   |
| `output_frame_id` |   字符串   |    合并后点云所在的坐标系    |



**额外需求**

用户必须从外部配置TF-tree，如将要把`/left_rslidar`下的点云与其他点云合并为`\base_link ` 坐标系下的`/concat_points`，那么在当前TF-Tree中必须存在一条可从`/left_rslidar`到`\base_link ` 的路径

