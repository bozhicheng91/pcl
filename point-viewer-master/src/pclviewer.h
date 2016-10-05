#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QProgressDialog>
#include <QFuture>
#include <QFutureWatcher>
#include <QtConcurrentRun>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

  public:
    /** @brief Constructor 构造函数, explicit 关键字表面该构造函数必须显式调用*/
       explicit
       PCLViewer (QWidget *parent = 0);

       /** @brief Destructor 析构函数*/
       ~PCLViewer ();
       
void poissonReconstruction();

  //设置信号槽
  public slots:

       //文件浏览函数
       void
       browseFileButtonPressed ();

          /** @brief Triggered whenever the "Load file" button is clicked 当"load file 按钮被点击时, 该函数被触发"*/
       void
       loadFileButtonPressed ();

        /** @brief Triggered whenever a button in the "Color mode" group is clicked 颜色选择函数 */
       void
       colorSelected ();

//
       void
       pSliderValueChanged (int value);

  protected:
      /** @brief The PCL visualizer object */
      //创建显示对象指针
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

      /** @brief The point cloud displayed */
      //创建点云数据指针
      PointCloudT::Ptr cloud_;
      PointCloud::Ptr cloud;
      pcl::PolygonMesh triangles; 


       void
       addOrUpdateCloud();

       void
       load ();

       int 
       loadAsync (int i);
       void
       paintCloud();

  private:
       /** @brief ui pointer */
       Ui::PCLViewer *ui;

       bool first_time;
       
       QString filename;

   
};

#endif // PCLVIEER_H
