#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include "regulatenormaldialog.h"

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <C:\Program Files\PCL 1.8.1\include\pcl-1.8\pcl\point_cloud.h>
#include <C:\Program Files\PCL 1.8.1\include\pcl-1.8\pcl\point_types.h>
#include <C:\Program Files\PCL 1.8.1\include\pcl-1.8\pcl\visualization/pcl_visualizer.h>
#include <C:\Program Files\PCL 1.8.1\include\pcl-1.8\pcl\io\pcd_io.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/

// VTK
#include <vtkRenderWindow.h>

namespace Ui {
class mainWindow;
}

class mainWindow : public QMainWindow
{
    Q_OBJECT

protected:
    // 类成员变�??
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> main_clouds;
    RegulateNormalDialog regulateNormalDialog;
public:
    explicit mainWindow(QWidget *parent = 0);
    ~mainWindow();

private slots:
    void on_openFileAction_triggered();

    void on_bgColorMenu_triggered();

    void on_pointCloudColorMenu_triggered();

    void on_translateToCentroidAction_triggered();

    void on_removeNanAction_triggered();

    void on_plane_detect_set_param_Action_triggered();

    void on_removeRedundantPointsAction_triggered();

    void on_regulateCoorAction_triggered();

    void on_load_param_action_triggered();

    void on_normalEstimateAction_triggered();

    void on_regulateNormalAction_triggered();

    void on_performRegulateAction_triggered();

    void on_removePointCloudAction_triggered();

    void on_createPSAction_triggered();

    void on_savePointNormalFileAction_triggered();

    void on_openPointCloudNormalFileAction_triggered();

    void on_filtPSAction_triggered();

    void on_segPSAction_triggered();

    void on_segPlaneAction_triggered();

    void on_regionGrowingAction_triggered();

    void on_mergePlanesAction_triggered();

    void on_polyPlanesAction_triggered();

    void on_postProcessAction_triggered();

    void on_runAgainAction_triggered();

    void on_autoPerformAction_triggered();

    void on_editPolyAction_triggered();

    void on_delPolyAction_triggered();

    void on_performDelAction_triggered();

    void on_savePolyDataAction_triggered();

    void on_enterPruneModeAction_triggered();

    void on_selCurPolyAction_triggered();

    void on_setFirstPointAction_triggered();

    void on_setSecondPointAction_triggered();

    void on_performPolyCutAction_triggered();

    void on_displayLineSegAction_triggered();

    void on_switchLineSegAction_triggered();

    void on_performLineSegDelAction_triggered();

    void on_loadPolyDataAction_triggered();

    void on_voxelGridFiltAction_triggered();

private:
    Ui::mainWindow *ui;
};

#endif // MAINWINDOW_H
