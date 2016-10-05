// based on http://pointclouds.org/documentation/tutorials/qt_colorize_cloud.php#qt-colorize-cloud

#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include "Eigen/Dense"

//构造函数
PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    //对界面进行初始化.安装我们在qt设计器里设计的样子将窗体画出来, 把我们在qt设计器里面定义的信号和槽建立起来. 也可以说,
    //setupui是我们画界面和写程序之间的桥梁
    ui->setupUi (this);
    //设置窗口标题
    this->setWindowTitle ("Point Cloud Viewer");
    //初始化存在点云指针
    cloud_.reset (new PointCloudT);
    cloud.reset(new PointCloud);

    // Set up the QVTK window
    //初始化pcl封装的vtk窗口
    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    //设置窗口的背景色
    viewer_->setBackgroundColor (0.0, 0.0, 0.7); // so that we can see black points
   
    //vtk和qt的结合, 进行图像显示
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer_->setShowFPS(false);
    // ui->qvtkWidget->update ();

    // io
    //设置文件的浏览和加载信号槽函数
    connect (ui->pushButton_browse, SIGNAL(clicked ()), this, SLOT(browseFileButtonPressed ()));
    connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));

    // point color
    //设置点云颜色设置信号槽函数
    // connect (ui->radioButton_Original, SIGNAL(clicked ()), this, SLOT(colorSelected()));
    connect (ui->radioButton_point, SIGNAL(clicked ()), this, SLOT(colorSelected()));
    connect (ui->radioButton_wireframe, SIGNAL(clicked ()), this, SLOT(colorSelected()));
    connect (ui->radioButton_surface, SIGNAL(clicked ()), this, SLOT(colorSelected()));
    // connect (ui->radioButton_Rainbow, SIGNAL(clicked ()), this, SLOT(colorSelected()));

    // point size
    //设置点云样点显示尺寸
    // connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    
    this->first_time = true;

    QString filename = ui->lineEdit_path->text();
    load(filename);//加载文件
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
    delete ui;
}

void
PCLViewer::load(QString &filename) 
{
    //qprogressdialog 进度对话框, 继承值QDialog类

    QProgressDialog dialog("Loading...", "Cancel", 0, 0, this);
    //设置进度对话框标题
    dialog.setWindowModality(Qt::WindowModal);
    dialog.setCancelButton(0); // since `QtConcurrent::run` cannot be canceled
    dialog.setValue(0);
    
    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

    QFutureWatcher<void> futureWatcher;

    QObject::connect(&futureWatcher, SIGNAL(finished()), &dialog, SLOT(reset()));
    QObject::connect(&dialog, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));

    // note: `run()` can not be canceled
    QFuture<void> future = QtConcurrent::run(this, &PCLViewer::loadAsync, filename);

    // start loading
    futureWatcher.setFuture(future);

    // show progress dialog
    dialog.exec();

    futureWatcher.waitForFinished();

}

void
PCLViewer::paintCloud()
{
    // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons

    viewer_->addPolygonMesh(triangles, "mesh");
    
    
    if (ui->radioButton_point->isChecked ())
        {
            PCL_INFO("point mode  chosen\n");
            viewer_->setRepresentationToPointsForAllActors(); 
 
        }
    else if (ui->radioButton_surface->isChecked ())
        {
            PCL_INFO("surface mode  chosen\n");
            viewer_->setRepresentationToSurfaceForAllActors(); 
        }
    else if (ui->radioButton_wireframe->isChecked ())
             {
                 PCL_INFO("wireframe mode  chosen\n");
                 viewer_->setRepresentationToWireframeForAllActors(); 
             }
    

}


// slots
//文件浏览对话框函数.
void
PCLViewer::browseFileButtonPressed ()
{
    
    QString dir;
    QFileInfo fi(ui->lineEdit_path->text());
    //若浏览的文件存在.
    if(fi.exists()) 
        {
            dir = fi.dir().path();
        } else {
        dir = "/home/";
    }

    //getOpenFileName 通过窗口选择要打开的文件, 并返回选择的文件完整路径和文件名.    
    QString filename = 
        QFileDialog::getOpenFileName (this, 
                                      tr ("Open point cloud"), 
                                      dir.toStdString().c_str(), 
                                      tr ("Point cloud data (*.pcd *.ply *vtk)"));

    if (filename.isEmpty ())
        return;
    
    ui->lineEdit_path->setText(filename);

}

void
PCLViewer::loadFileButtonPressed ()
{
    QString filename = ui->lineEdit_path->text();
    load(filename);
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();
}

void
PCLViewer::colorSelected ()
{
    paintCloud();
    ui->qvtkWidget->update ();
}

void
PCLViewer::pSliderValueChanged (int value)
{
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
    ui->qvtkWidget->update ();
}

// helpers

void
PCLViewer::addOrUpdateCloud(pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGBA> &handler)
{
    if(first_time) 
        {


            poissonReconstruction();
            //viewer_->addPointCloud (cloud_, handler, "cloud");
            viewer_->addPolygonMesh(triangles, "mesh");
            first_time = false;
        } 
    else
        {
            // viewer_->updatePointCloud (cloud_, handler, "cloud");
            viewer_->addPolygonMesh(triangles, "mesh");
        }
}

int 
PCLViewer::loadAsync(QString &filename) 
{

      std::cerr << "开始加载点云数据 ....." << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename.toStdString (), *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //若读取失败将提示
            return -1;
        }
    std::cerr << "点云读入   完成" << std::endl;
    poissonReconstruction();
    paintCloud();
    /*
    PointCloudT::Ptr cloud_tmp (new PointCloudT);
    PointCloud::Ptr cloud1_tmp(new PointCloud);

    int return_status;

    if (filename.endsWith (".ply", Qt::CaseInsensitive))
        {
            return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);

            // If point cloud contains NaN values, remove them before updating the visualizer point cloud
            if (cloud_tmp->is_dense)
                {
                    pcl::copyPointCloud (*cloud_tmp, *cloud_);
                    // pcl::copyPointCloud (*cloud_tmp, *cloud_);
                }
            else
                {
                    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
                    std::vector<int> vec;
                    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
                }

            paintCloud();
            
        }
    else if (filename.endsWith (".pcd", Qt::CaseInsensitive)){
        return_status = pcl::io::loadPCDFile<Point> (filename, *cloud);

 std::cerr << "File load done!!" << std::endl;
       
        if (cloud1_tmp->is_dense)
            {
                pcl::copyPointCloud (*cloud1_tmp, *cloud);
               
            }
        else
            {
                PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
                std::vector<int> vec;
                pcl::removeNaNFromPointCloud (*cloud1_tmp, *cloud, vec);
            }

        paintCloud();
    }
   
    else return_status = 1;



    if (return_status != 0)
        {
            PCL_ERROR("Error reading data %s\n", filename.toStdString ().c_str ());
            return;
        }
    */
    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  
}

void
PCLViewer::poissonReconstruction()
{

    /*法向估计模块*/
    // Normal estimation（法向量估计）
    std::cerr << "开始poisson重建" <<std::endl;
    pcl::NormalEstimation<Point, pcl::Normal> n;//创建法向估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//创建法向数据指针
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);//创建kdtree用于法向计算时近邻搜索
    tree->setInputCloud(cloud);//为kdtree输入点云
    n.setInputCloud(cloud);//为法向估计对象输入点云
    n.setSearchMethod(tree);//设置法向估计时采用的搜索方式为kdtree
    n.setKSearch(20);//设置法向估计时,k近邻搜索的点数
    n.compute(*normals);  //进行法向估计
    
    std::cerr << "法线计算   完成" << std::endl;

    /*点云数据与法向数据拼接*/
    // 创建同时包含点和法向的数据结构的指针
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    //将已获得的点数据和法向数据拼接
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);


    // 创建另一个kdtree用于重建
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    //为kdtree输入点云数据,该点云数据类型为点和法向
    tree2->setInputCloud(cloud_with_normals);

    /*曲面重建模块*/
    // 创建贪婪三角形投影重建对象
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //创建多边形网格对象,用来存储重建结果
  

    //设置参数
    gp3.setSearchRadius(25);  // 设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径（默认为0）
    gp3.setMu(2.5);  // 设置最近邻距离的乘子，已得到每个点的最终搜索半径（默认为0）
    gp3.setMaximumNearestNeighbors(100);  //设置搜索的最近邻点的最大数量
    gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees 最大平面角
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees 每个三角的最大角度
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);  //若法向量一致，设为true
    // 设置点云数据和搜索方式
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    //开始重建
    gp3.reconstruct(triangles);
    std::cerr << "重建   完成" << std::endl;

    //将重建结果保存到硬盘文件中,重建结果以VTK格式存储
    pcl::io::saveVTKFile("mymesh.vtk", triangles); 


}

