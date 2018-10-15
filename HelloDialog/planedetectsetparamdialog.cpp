#include "PlaneDetectSetParamDialog.h"
#include "ui_PlaneDetectSetParamDialog.h"
#include <QMessageBox>
#include <QStringList>
#include <iostream>

PlaneDetectSetParamDialog::PlaneDetectSetParamDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PlaneDetectSetParamDialog)
{
    ui->setupUi(this);    
    model = new QStandardItemModel();
    initTable();
}

PlaneDetectSetParamDialog::~PlaneDetectSetParamDialog()
{
    delete ui;
    delete model;
    delete configIniRead;
}

void PlaneDetectSetParamDialog::initTable()
{
    //ui->tableView->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    //ui->tableView->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    //ui->tableView->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
    // 设置表格每列的宽度
    ui->tableView->setColumnWidth(0, 100);
    ui->tableView->setColumnWidth(1, 600);
    ui->tableView->setColumnWidth(2, 50);

    // 设置表头
    model->setHorizontalHeaderItem(0, new QStandardItem(QObject::tr("variable name")));
    model->setHorizontalHeaderItem(1, new QStandardItem(QObject::tr("role")));
    model->setHorizontalHeaderItem(2, new QStandardItem(QObject::tr("value")));
    ui->tableView->setModel(model);

    configIniRead = new QSettings("config.txt", QSettings::IniFormat);
    // 设置表项
	QString information0 = information0.fromLocal8Bit("是否对点云执行平移操作");
	QString information1 = information1.fromLocal8Bit("去除同一个位置的冗余点时使用，也就是两点之间的最小距离");
	QString information2 = information2.fromLocal8Bit("计算法向量的搜索半径");
	QString information3 = information3.fromLocal8Bit("点云法向量的显示间隔");
	QString information4 = information4.fromLocal8Bit("点云法向量的显示长度");
	QString information5 = information5.fromLocal8Bit("选中点的法向量的长度");
	QString information6 = information6.fromLocal8Bit("法向量是否指向物体外部");
	QString information7 = information7.fromLocal8Bit("校正法向量方向的搜索半径");
	QString information8 = information8.fromLocal8Bit("参数空间数值分辨率");
	QString information9 = information9.fromLocal8Bit("可视化投票情况的颜色增益");
	QString information10 = information10.fromLocal8Bit("高斯核函数的标准差");
	QString information11 = information11.fromLocal8Bit("创建梯度场时的邻域搜索半径");
	QString information12 = information12.fromLocal8Bit("向sink点进行投票的最小数量阈值");
	QString information13 = information13.fromLocal8Bit("梯度场sink点校正时初始的搜索半径");
	QString information14 = information14.fromLocal8Bit("radius_base的增量");
	QString information15 = information15.fromLocal8Bit("有效单元占比的阈值");
	QString information16 = information16.fromLocal8Bit("校正梯度场时的角度偏离阈值(角度）");
	QString information17 = information17.fromLocal8Bit("进行平面分割时搜索半径");
	QString information18 = information18.fromLocal8Bit("单个平面点数量阈值");
	QString information19 = information19.fromLocal8Bit("实现平面区域扩充时，寻找边界的半径");
	QString information20 = information20.fromLocal8Bit("实现平面区域扩充时，寻找局部平面的半径");
	QString information21 = information21.fromLocal8Bit("更新后平面的角度偏离阈值(相对于种子平面)（角度）");
	QString information22 = information22.fromLocal8Bit("进行平面增长时计算点的法向量时选取的一个较小的搜索半径");
	QString information23 = information23.fromLocal8Bit("曲率偏移阈值");
	QString information24 = information24.fromLocal8Bit("平面边缘点的法向量与平面法向量的偏离阈值");
	QString information25 = information25.fromLocal8Bit("进行平面合并时公共部分的最小比例");
	QString information26 = information26.fromLocal8Bit("构造平面凹包时alpha大小");
	QString information27 = information27.fromLocal8Bit("点与平面的距离阈值");
	QString information28 = information28.fromLocal8Bit("每个聚类的最小数量阈值");

	setTableItem(0, "isConductTranslate", information0);
	setTableItem(1, "min_dist_between_points", information1);
	setTableItem(2, "r_for_estimate_normal", information2);
	setTableItem(3, "interval_level", information3);
	setTableItem(4, "normal_length_for_display", information4);
	setTableItem(5, "normal_length_for_selected", information5);
	setTableItem(6, "is_norm_direction_valid", information6);
	setTableItem(7, "r_for_regulate_normal", information7);
	setTableItem(8, "num_res", information8);
	setTableItem(9, "color_gain", information9);
	setTableItem(10, "std_dev_gaussian", information10);
	setTableItem(11, "search_radius_create_gradient", information11);
	setTableItem(12, "T_vote_num", information12);
	setTableItem(13, "radius_base", information13);
	setTableItem(14, "delta_radius", information14);
	setTableItem(15, "S_threshold", information15);
	setTableItem(16, "dev_threshold", information16);
	setTableItem(17, "search_radius_for_plane_seg", information17);
	setTableItem(18, "T_num_of_single_plane", information18);
	setTableItem(19, "radius_border", information19);
	setTableItem(20, "radius_local", information20);
	setTableItem(21, "T_angle_bias_integrate", information21);
	setTableItem(22, "r_local", information22);
	setTableItem(23, "T_curvature_bias", information23);
	setTableItem(24, "T_point_normal_bias", information24);
	setTableItem(25, "T_ratio_merge_planes", information25);
	setTableItem(26, "alpha_poly", information26);
	setTableItem(27, "T_dist_point_plane", information27);
	setTableItem(28, "T_cluster_num", information28);
}

void PlaneDetectSetParamDialog::setTableItem(int row, QString str1, QString str2)
{
    model->setItem(row,0, new QStandardItem(str1));
    model->setItem(row,1, new QStandardItem(str2));
    QString str3 = "PlaneDetect/";
    str3.append(str1);
    model->setItem(row,2, new QStandardItem(configIniRead->value(str3).toString()));
}

void PlaneDetectSetParamDialog::on_buttonBox_accepted()
{
    QStringList strList;
    strList << "isConductTranslate"
            << "min_dist_between_points"
            << "r_for_estimate_normal"
            << "interval_level"
            << "normal_length_for_display"
            << "normal_length_for_selected"
            << "is_norm_direction_valid"
            << "r_for_regulate_normal"
            << "num_res"
            << "color_gain"
            << "std_dev_gaussian"
            << "search_radius_create_gradient"
            << "T_vote_num"
            << "radius_base"
            << "delta_radius"
            << "S_threshold"
            << "dev_threshold"
            << "search_radius_for_plane_seg"
            << "T_num_of_single_plane"
            << "radius_border"
            << "radius_local"
            << "T_angle_bias_integrate"
            << "r_local"
            << "T_curvature_bias"
            << "T_point_normal_bias"
            << "T_ratio_merge_planes"
            << "alpha_poly"
            << "T_dist_point_plane"
            << "T_cluster_num";
    int size = strList.size();
    for(int i = 0; i < size; ++i)
    {
        QString key, value;
        key = model->data(model->index(i, 0), Qt::DisplayRole).toString();
        value = model->data(model->index(i, 2), Qt::DisplayRole).toString();
        key = "PlaneDetect/" + key;
        configIniRead->setValue(key, value);
    }

	QString title = title.fromLocal8Bit("提示");
	QString information = information.fromLocal8Bit("参数已设置好");
	QMessageBox::about(NULL, title, information);
}




















