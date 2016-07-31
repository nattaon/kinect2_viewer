#include "mainwindow.h"
#include "build/ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	//connect ui to function
	connect(ui->bt_startcam, SIGNAL(clicked()), this, SLOT(ButtonStartCameraPressed()));
	connect(ui->bt_stopcam, SIGNAL(clicked()), this, SLOT(ButtonStopCameraPressed()));

	//pcl viewer
	viewer.reset(new pcl::visualization::PCLVisualizer("kinect2 viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::ButtonStartCameraPressed()
{
	pointcloud.reset(new PointCloudXYZRGB);

	kinect = new KinectV2Interface();
	int return_val = kinect->openKinect();

	timerId = startTimer(100); //call timerEvemt every 100 msec
	time.start(); 
}

void MainWindow::ButtonStopCameraPressed()
{
	killTimer(timerId);

	viewer->removeAllPointClouds();
	viewer->spinOnce(100);

	cv::destroyWindow("kinect");
	delete kinect;
}

void MainWindow::timerEvent(QTimerEvent *event)
{
	cv::namedWindow("kinect");
	cv::Mat colorMat = kinect->get_colorframe(0.3);
	cv::imshow("kinect", colorMat);

	kinect->get_depthframe();
	kinect->mapping_pointcloud(pointcloud);

	if (!viewer->updatePointCloud(pointcloud, "pointcloud"))
	{
		viewer->addPointCloud(pointcloud, "pointcloud");
	}
	viewer->spinOnce(100);
}