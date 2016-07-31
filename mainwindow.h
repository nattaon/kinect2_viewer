#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "KinectV2Interface.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QMainWindow>
#include <QTime>
typedef pcl::PointXYZRGB PointTypeXYZRGB;
typedef pcl::PointCloud<PointTypeXYZRGB> PointCloudXYZRGB;
using namespace std;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PointCloudXYZRGB::Ptr pointcloud;	
	KinectV2Interface *kinect;	
	QTime time;
	int timerId;

	void timerEvent(QTimerEvent *event);

private slots:
	void ButtonStartCameraPressed();
	void ButtonStopCameraPressed();
};

#endif // MAINWINDOW_H
