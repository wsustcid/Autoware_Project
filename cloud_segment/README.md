<!--
 * @Author: Shuai Wang
 * @Github: https://github.com/wsustcid
 * @Version: 0.0.0
 * @Date: 2022-04-16 21:55:03
 * @LastEditTime: 2022-04-17 11:39:24
-->

# Cloud Segmentation
给定点云数据，基于欧式聚类，实现对车辆的分割，并借助可视化工具将分割结果以boundingbox的形式显示出来，主要处理过程包括：滤波、切割、去地面、聚类、可视化等步骤。

将聚类得到的目标前后两帧进行跟踪，获取动态目标，对动态目标框处bounding box
