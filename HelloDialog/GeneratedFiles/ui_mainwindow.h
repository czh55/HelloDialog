/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionfile;
    QAction *openFileAction;
    QAction *openPointCloudNormalFileAction;
    QAction *loadPolyDataAction;
    QAction *bgColorMenu;
    QAction *pointCloudColorMenu;
    QAction *removePointCloudAction;
    QAction *removeNanAction;
    QAction *removeRedundantPointsAction;
    QAction *translateToCentroidAction;
    QAction *regulateCoorAction;
    QAction *voxelGridFiltAction;
    QAction *plane_detect_set_param_Action;
    QAction *three_d_reconstruct_Action;
    QAction *load_param_action;
    QAction *regionGrowingAction;
    QAction *mergePlanesAction;
    QAction *postProcessAction;
    QAction *runAgainAction;
    QAction *autoPerformAction;
    QAction *savePolyDataAction;
    QAction *normalEstimateAction;
    QAction *savePointNormalFileAction;
    QAction *regulateNormalAction;
    QAction *performRegulateAction;
    QAction *createPSAction;
    QAction *filtPSAction;
    QAction *segPSAction;
    QAction *segPlaneAction;
    QAction *editPolyAction;
    QAction *delPolyAction;
    QAction *performDelAction;
    QAction *enterPruneModeAction;
    QAction *selCurPolyAction;
    QAction *setFirstPointAction;
    QAction *setSecondPointAction;
    QAction *performPolyCutAction;
    QAction *displayLineSegAction;
    QAction *switchLineSegAction;
    QAction *performLineSegDelAction;
    QAction *polyPlanesAction_2;
    QAction *openFile1_RegistrationAction;
    QAction *openFile2_RegistrationAction;
    QAction *registrationICPAction;
    QAction *action;
    QAction *repairHolesAction;
    QAction *openFile1RegistrationAction;
    QAction *openFile2RegistrationAction;
    QAction *registrationSACAction;
    QAction *rotatePointCloudAction;
    QAction *registrationPlaneAction;
    QAction *voxelGridFiltMergeCloudAction;
    QAction *action_3;
    QAction *cleanPointCloudAction;
    QAction *removeNan1Action;
    QAction *voxelGridFilt1Action;
    QAction *savePointCloudAction;
    QAction *repairHolesOFFAction;
    QAction *TriangularMeshingAction;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QVTKWidget *qvtkWidget;
    QMenuBar *menuBar;
    QMenu *fileMenu;
    QMenu *renderPropertyMenu;
    QMenu *pointCloudPreProcessMenu;
    QMenu *param_set_menu;
    QMenu *plane_detect_menu;
    QMenu *normalProcessMenu;
    QMenu *regulateNormalMenu;
    QMenu *psMenu;
    QMenu *segmentMenu;
    QMenu *editPolyMenu;
    QMenu *delLineSegMenu;
    QMenu *registrationMenu;
    QMenu *repairHolesMenu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(600, 424);
        MainWindow->setLayoutDirection(Qt::LeftToRight);
        MainWindow->setAutoFillBackground(true);
        actionfile = new QAction(MainWindow);
        actionfile->setObjectName(QStringLiteral("actionfile"));
        openFileAction = new QAction(MainWindow);
        openFileAction->setObjectName(QStringLiteral("openFileAction"));
        openPointCloudNormalFileAction = new QAction(MainWindow);
        openPointCloudNormalFileAction->setObjectName(QStringLiteral("openPointCloudNormalFileAction"));
        loadPolyDataAction = new QAction(MainWindow);
        loadPolyDataAction->setObjectName(QStringLiteral("loadPolyDataAction"));
        bgColorMenu = new QAction(MainWindow);
        bgColorMenu->setObjectName(QStringLiteral("bgColorMenu"));
        pointCloudColorMenu = new QAction(MainWindow);
        pointCloudColorMenu->setObjectName(QStringLiteral("pointCloudColorMenu"));
        removePointCloudAction = new QAction(MainWindow);
        removePointCloudAction->setObjectName(QStringLiteral("removePointCloudAction"));
        removeNanAction = new QAction(MainWindow);
        removeNanAction->setObjectName(QStringLiteral("removeNanAction"));
        removeRedundantPointsAction = new QAction(MainWindow);
        removeRedundantPointsAction->setObjectName(QStringLiteral("removeRedundantPointsAction"));
        translateToCentroidAction = new QAction(MainWindow);
        translateToCentroidAction->setObjectName(QStringLiteral("translateToCentroidAction"));
        regulateCoorAction = new QAction(MainWindow);
        regulateCoorAction->setObjectName(QStringLiteral("regulateCoorAction"));
        voxelGridFiltAction = new QAction(MainWindow);
        voxelGridFiltAction->setObjectName(QStringLiteral("voxelGridFiltAction"));
        plane_detect_set_param_Action = new QAction(MainWindow);
        plane_detect_set_param_Action->setObjectName(QStringLiteral("plane_detect_set_param_Action"));
        three_d_reconstruct_Action = new QAction(MainWindow);
        three_d_reconstruct_Action->setObjectName(QStringLiteral("three_d_reconstruct_Action"));
        load_param_action = new QAction(MainWindow);
        load_param_action->setObjectName(QStringLiteral("load_param_action"));
        regionGrowingAction = new QAction(MainWindow);
        regionGrowingAction->setObjectName(QStringLiteral("regionGrowingAction"));
        mergePlanesAction = new QAction(MainWindow);
        mergePlanesAction->setObjectName(QStringLiteral("mergePlanesAction"));
        postProcessAction = new QAction(MainWindow);
        postProcessAction->setObjectName(QStringLiteral("postProcessAction"));
        runAgainAction = new QAction(MainWindow);
        runAgainAction->setObjectName(QStringLiteral("runAgainAction"));
        autoPerformAction = new QAction(MainWindow);
        autoPerformAction->setObjectName(QStringLiteral("autoPerformAction"));
        savePolyDataAction = new QAction(MainWindow);
        savePolyDataAction->setObjectName(QStringLiteral("savePolyDataAction"));
        normalEstimateAction = new QAction(MainWindow);
        normalEstimateAction->setObjectName(QStringLiteral("normalEstimateAction"));
        savePointNormalFileAction = new QAction(MainWindow);
        savePointNormalFileAction->setObjectName(QStringLiteral("savePointNormalFileAction"));
        regulateNormalAction = new QAction(MainWindow);
        regulateNormalAction->setObjectName(QStringLiteral("regulateNormalAction"));
        performRegulateAction = new QAction(MainWindow);
        performRegulateAction->setObjectName(QStringLiteral("performRegulateAction"));
        createPSAction = new QAction(MainWindow);
        createPSAction->setObjectName(QStringLiteral("createPSAction"));
        filtPSAction = new QAction(MainWindow);
        filtPSAction->setObjectName(QStringLiteral("filtPSAction"));
        segPSAction = new QAction(MainWindow);
        segPSAction->setObjectName(QStringLiteral("segPSAction"));
        segPlaneAction = new QAction(MainWindow);
        segPlaneAction->setObjectName(QStringLiteral("segPlaneAction"));
        editPolyAction = new QAction(MainWindow);
        editPolyAction->setObjectName(QStringLiteral("editPolyAction"));
        delPolyAction = new QAction(MainWindow);
        delPolyAction->setObjectName(QStringLiteral("delPolyAction"));
        performDelAction = new QAction(MainWindow);
        performDelAction->setObjectName(QStringLiteral("performDelAction"));
        enterPruneModeAction = new QAction(MainWindow);
        enterPruneModeAction->setObjectName(QStringLiteral("enterPruneModeAction"));
        selCurPolyAction = new QAction(MainWindow);
        selCurPolyAction->setObjectName(QStringLiteral("selCurPolyAction"));
        setFirstPointAction = new QAction(MainWindow);
        setFirstPointAction->setObjectName(QStringLiteral("setFirstPointAction"));
        setSecondPointAction = new QAction(MainWindow);
        setSecondPointAction->setObjectName(QStringLiteral("setSecondPointAction"));
        performPolyCutAction = new QAction(MainWindow);
        performPolyCutAction->setObjectName(QStringLiteral("performPolyCutAction"));
        displayLineSegAction = new QAction(MainWindow);
        displayLineSegAction->setObjectName(QStringLiteral("displayLineSegAction"));
        switchLineSegAction = new QAction(MainWindow);
        switchLineSegAction->setObjectName(QStringLiteral("switchLineSegAction"));
        performLineSegDelAction = new QAction(MainWindow);
        performLineSegDelAction->setObjectName(QStringLiteral("performLineSegDelAction"));
        polyPlanesAction_2 = new QAction(MainWindow);
        polyPlanesAction_2->setObjectName(QStringLiteral("polyPlanesAction_2"));
        openFile1_RegistrationAction = new QAction(MainWindow);
        openFile1_RegistrationAction->setObjectName(QStringLiteral("openFile1_RegistrationAction"));
        openFile2_RegistrationAction = new QAction(MainWindow);
        openFile2_RegistrationAction->setObjectName(QStringLiteral("openFile2_RegistrationAction"));
        registrationICPAction = new QAction(MainWindow);
        registrationICPAction->setObjectName(QStringLiteral("registrationICPAction"));
        action = new QAction(MainWindow);
        action->setObjectName(QStringLiteral("action"));
        repairHolesAction = new QAction(MainWindow);
        repairHolesAction->setObjectName(QStringLiteral("repairHolesAction"));
        openFile1RegistrationAction = new QAction(MainWindow);
        openFile1RegistrationAction->setObjectName(QStringLiteral("openFile1RegistrationAction"));
        openFile2RegistrationAction = new QAction(MainWindow);
        openFile2RegistrationAction->setObjectName(QStringLiteral("openFile2RegistrationAction"));
        registrationSACAction = new QAction(MainWindow);
        registrationSACAction->setObjectName(QStringLiteral("registrationSACAction"));
        rotatePointCloudAction = new QAction(MainWindow);
        rotatePointCloudAction->setObjectName(QStringLiteral("rotatePointCloudAction"));
        registrationPlaneAction = new QAction(MainWindow);
        registrationPlaneAction->setObjectName(QStringLiteral("registrationPlaneAction"));
        voxelGridFiltMergeCloudAction = new QAction(MainWindow);
        voxelGridFiltMergeCloudAction->setObjectName(QStringLiteral("voxelGridFiltMergeCloudAction"));
        action_3 = new QAction(MainWindow);
        action_3->setObjectName(QStringLiteral("action_3"));
        cleanPointCloudAction = new QAction(MainWindow);
        cleanPointCloudAction->setObjectName(QStringLiteral("cleanPointCloudAction"));
        removeNan1Action = new QAction(MainWindow);
        removeNan1Action->setObjectName(QStringLiteral("removeNan1Action"));
        voxelGridFilt1Action = new QAction(MainWindow);
        voxelGridFilt1Action->setObjectName(QStringLiteral("voxelGridFilt1Action"));
        savePointCloudAction = new QAction(MainWindow);
        savePointCloudAction->setObjectName(QStringLiteral("savePointCloudAction"));
        repairHolesOFFAction = new QAction(MainWindow);
        repairHolesOFFAction->setObjectName(QStringLiteral("repairHolesOFFAction"));
        TriangularMeshingAction = new QAction(MainWindow);
        TriangularMeshingAction->setObjectName(QStringLiteral("TriangularMeshingAction"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        centralWidget->setLayoutDirection(Qt::LeftToRight);
        centralWidget->setAutoFillBackground(true);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setLayoutDirection(Qt::LeftToRight);

        horizontalLayout->addWidget(qvtkWidget);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        fileMenu = new QMenu(menuBar);
        fileMenu->setObjectName(QStringLiteral("fileMenu"));
        renderPropertyMenu = new QMenu(menuBar);
        renderPropertyMenu->setObjectName(QStringLiteral("renderPropertyMenu"));
        pointCloudPreProcessMenu = new QMenu(menuBar);
        pointCloudPreProcessMenu->setObjectName(QStringLiteral("pointCloudPreProcessMenu"));
        param_set_menu = new QMenu(menuBar);
        param_set_menu->setObjectName(QStringLiteral("param_set_menu"));
        plane_detect_menu = new QMenu(menuBar);
        plane_detect_menu->setObjectName(QStringLiteral("plane_detect_menu"));
        normalProcessMenu = new QMenu(plane_detect_menu);
        normalProcessMenu->setObjectName(QStringLiteral("normalProcessMenu"));
        regulateNormalMenu = new QMenu(plane_detect_menu);
        regulateNormalMenu->setObjectName(QStringLiteral("regulateNormalMenu"));
        psMenu = new QMenu(plane_detect_menu);
        psMenu->setObjectName(QStringLiteral("psMenu"));
        segmentMenu = new QMenu(plane_detect_menu);
        segmentMenu->setObjectName(QStringLiteral("segmentMenu"));
        editPolyMenu = new QMenu(plane_detect_menu);
        editPolyMenu->setObjectName(QStringLiteral("editPolyMenu"));
        delLineSegMenu = new QMenu(editPolyMenu);
        delLineSegMenu->setObjectName(QStringLiteral("delLineSegMenu"));
        registrationMenu = new QMenu(menuBar);
        registrationMenu->setObjectName(QStringLiteral("registrationMenu"));
        repairHolesMenu = new QMenu(menuBar);
        repairHolesMenu->setObjectName(QStringLiteral("repairHolesMenu"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(fileMenu->menuAction());
        menuBar->addAction(renderPropertyMenu->menuAction());
        menuBar->addAction(pointCloudPreProcessMenu->menuAction());
        menuBar->addAction(registrationMenu->menuAction());
        menuBar->addAction(repairHolesMenu->menuAction());
        menuBar->addAction(param_set_menu->menuAction());
        menuBar->addAction(plane_detect_menu->menuAction());
        fileMenu->addAction(openFileAction);
        fileMenu->addAction(openPointCloudNormalFileAction);
        fileMenu->addAction(loadPolyDataAction);
        renderPropertyMenu->addAction(bgColorMenu);
        renderPropertyMenu->addAction(pointCloudColorMenu);
        renderPropertyMenu->addAction(cleanPointCloudAction);
        pointCloudPreProcessMenu->addAction(removeNan1Action);
        pointCloudPreProcessMenu->addAction(removeRedundantPointsAction);
        pointCloudPreProcessMenu->addAction(translateToCentroidAction);
        pointCloudPreProcessMenu->addAction(regulateCoorAction);
        pointCloudPreProcessMenu->addAction(voxelGridFilt1Action);
        pointCloudPreProcessMenu->addAction(savePointCloudAction);
        param_set_menu->addAction(plane_detect_set_param_Action);
        param_set_menu->addAction(load_param_action);
        plane_detect_menu->addAction(normalProcessMenu->menuAction());
        plane_detect_menu->addAction(regulateNormalMenu->menuAction());
        plane_detect_menu->addAction(psMenu->menuAction());
        plane_detect_menu->addAction(segmentMenu->menuAction());
        plane_detect_menu->addAction(regionGrowingAction);
        plane_detect_menu->addAction(mergePlanesAction);
        plane_detect_menu->addAction(polyPlanesAction_2);
        plane_detect_menu->addAction(postProcessAction);
        plane_detect_menu->addSeparator();
        plane_detect_menu->addAction(runAgainAction);
        plane_detect_menu->addSeparator();
        plane_detect_menu->addAction(autoPerformAction);
        plane_detect_menu->addSeparator();
        plane_detect_menu->addAction(editPolyMenu->menuAction());
        plane_detect_menu->addSeparator();
        plane_detect_menu->addAction(savePolyDataAction);
        normalProcessMenu->addAction(normalEstimateAction);
        normalProcessMenu->addAction(savePointNormalFileAction);
        regulateNormalMenu->addAction(regulateNormalAction);
        regulateNormalMenu->addAction(performRegulateAction);
        psMenu->addAction(createPSAction);
        psMenu->addAction(filtPSAction);
        segmentMenu->addAction(segPSAction);
        segmentMenu->addAction(segPlaneAction);
        editPolyMenu->addAction(editPolyAction);
        editPolyMenu->addAction(delPolyAction);
        editPolyMenu->addAction(performDelAction);
        editPolyMenu->addSeparator();
        editPolyMenu->addAction(enterPruneModeAction);
        editPolyMenu->addAction(selCurPolyAction);
        editPolyMenu->addAction(setFirstPointAction);
        editPolyMenu->addAction(setSecondPointAction);
        editPolyMenu->addAction(performPolyCutAction);
        editPolyMenu->addAction(delLineSegMenu->menuAction());
        delLineSegMenu->addAction(displayLineSegAction);
        delLineSegMenu->addAction(switchLineSegAction);
        delLineSegMenu->addAction(performLineSegDelAction);
        registrationMenu->addAction(openFile1RegistrationAction);
        registrationMenu->addAction(openFile2RegistrationAction);
        registrationMenu->addSeparator();
        registrationMenu->addAction(removeNanAction);
        registrationMenu->addAction(voxelGridFiltAction);
        registrationMenu->addSeparator();
        registrationMenu->addAction(rotatePointCloudAction);
        registrationMenu->addSeparator();
        registrationMenu->addAction(registrationSACAction);
        registrationMenu->addAction(registrationICPAction);
        registrationMenu->addSeparator();
        registrationMenu->addAction(registrationPlaneAction);
        registrationMenu->addSeparator();
        registrationMenu->addAction(voxelGridFiltMergeCloudAction);
        repairHolesMenu->addAction(TriangularMeshingAction);
        repairHolesMenu->addAction(repairHolesOFFAction);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        actionfile->setText(QApplication::translate("MainWindow", "file", Q_NULLPTR));
        openFileAction->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200\347\202\271\344\272\221\346\226\207\344\273\266(&O)", Q_NULLPTR));
        openPointCloudNormalFileAction->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200\347\202\271\344\272\221\346\263\225\345\220\221\351\207\217\346\226\207\344\273\266", Q_NULLPTR));
        loadPolyDataAction->setText(QApplication::translate("MainWindow", "\345\212\240\350\275\275\345\244\232\350\276\271\345\275\242\346\225\260\346\215\256", Q_NULLPTR));
        bgColorMenu->setText(QApplication::translate("MainWindow", "\350\203\214\346\231\257\350\211\262", Q_NULLPTR));
        pointCloudColorMenu->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\351\242\234\350\211\262", Q_NULLPTR));
        removePointCloudAction->setText(QApplication::translate("MainWindow", "\347\247\273\351\231\244\347\202\271\344\272\221", Q_NULLPTR));
        removeNanAction->setText(QApplication::translate("MainWindow", "\344\270\244\347\202\271\344\272\221\345\210\206\345\210\253\347\247\273\351\231\244NAN\347\202\271", Q_NULLPTR));
        removeRedundantPointsAction->setText(QApplication::translate("MainWindow", "\345\216\273\351\231\244\345\206\227\344\275\231\347\202\271", Q_NULLPTR));
        translateToCentroidAction->setText(QApplication::translate("MainWindow", "\345\271\263\347\247\273\345\210\260\351\207\215\345\277\203", Q_NULLPTR));
        regulateCoorAction->setText(QApplication::translate("MainWindow", "\344\277\256\346\255\243\345\235\220\346\240\207", Q_NULLPTR));
        voxelGridFiltAction->setText(QApplication::translate("MainWindow", "\344\270\244\347\202\271\344\272\221\345\210\206\345\210\253\344\275\223\347\264\240\346\273\244\346\263\242", Q_NULLPTR));
        plane_detect_set_param_Action->setText(QApplication::translate("MainWindow", "\345\217\202\346\225\260\350\256\276\347\275\256", Q_NULLPTR));
        three_d_reconstruct_Action->setText(QApplication::translate("MainWindow", "\344\270\211\347\273\264\351\207\215\345\273\272", Q_NULLPTR));
        load_param_action->setText(QApplication::translate("MainWindow", "\345\212\240\350\275\275\345\217\202\346\225\260", Q_NULLPTR));
        regionGrowingAction->setText(QApplication::translate("MainWindow", "\345\214\272\345\237\237\347\224\237\351\225\277", Q_NULLPTR));
        mergePlanesAction->setText(QApplication::translate("MainWindow", "\345\271\263\351\235\242\345\220\210\345\271\266", Q_NULLPTR));
        postProcessAction->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\345\220\216\345\244\204\347\220\206", Q_NULLPTR));
        runAgainAction->setText(QApplication::translate("MainWindow", "\347\273\247\347\273\255\350\277\220\350\241\214", Q_NULLPTR));
        autoPerformAction->setText(QApplication::translate("MainWindow", "\350\207\252\345\212\250\346\211\247\350\241\214", Q_NULLPTR));
        savePolyDataAction->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\345\244\232\350\276\271\345\275\242\346\225\260\346\215\256", Q_NULLPTR));
        normalEstimateAction->setText(QApplication::translate("MainWindow", "\344\274\260\350\256\241\346\263\225\345\220\221\351\207\217", Q_NULLPTR));
        savePointNormalFileAction->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\346\263\225\345\220\221\351\207\217\347\202\271\344\272\221\346\226\207\344\273\266", Q_NULLPTR));
        regulateNormalAction->setText(QApplication::translate("MainWindow", "\351\200\211\346\213\251\346\214\207\347\244\272\347\202\271", Q_NULLPTR));
        performRegulateAction->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\346\240\241\346\255\243", Q_NULLPTR));
        createPSAction->setText(QApplication::translate("MainWindow", "\345\210\233\345\273\272\345\217\202\346\225\260\347\251\272\351\227\264", Q_NULLPTR));
        filtPSAction->setText(QApplication::translate("MainWindow", "\345\271\263\346\273\221\345\217\202\346\225\260\347\251\272\351\227\264", Q_NULLPTR));
        segPSAction->setText(QApplication::translate("MainWindow", "\345\217\202\346\225\260\347\251\272\351\227\264", Q_NULLPTR));
        segPlaneAction->setText(QApplication::translate("MainWindow", "\345\271\263\351\235\242", Q_NULLPTR));
        editPolyAction->setText(QApplication::translate("MainWindow", "\350\277\233\345\205\245\345\244\232\350\276\271\345\275\242\345\210\240\351\231\244\346\250\241\345\274\217", Q_NULLPTR));
        delPolyAction->setText(QApplication::translate("MainWindow", "\347\241\256\345\256\232\345\210\240\351\231\244", Q_NULLPTR));
        performDelAction->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\345\210\240\351\231\244\346\223\215\344\275\234", Q_NULLPTR));
        enterPruneModeAction->setText(QApplication::translate("MainWindow", "\350\277\233\345\205\245\345\244\232\350\276\271\345\275\242\344\277\256\345\211\252\346\250\241\345\274\217", Q_NULLPTR));
        selCurPolyAction->setText(QApplication::translate("MainWindow", "\351\200\211\344\270\255\345\275\223\345\211\215\345\244\232\350\276\271\345\275\242", Q_NULLPTR));
        setFirstPointAction->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256\351\200\211\344\270\255\347\202\271\344\270\272\347\254\254\344\270\200\344\270\252\351\241\266\347\202\271", Q_NULLPTR));
        setSecondPointAction->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256\351\200\211\344\270\255\347\202\271\344\270\272\347\254\254\344\272\214\344\270\252\351\241\266\347\202\271", Q_NULLPTR));
        performPolyCutAction->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\345\244\232\350\276\271\345\275\242\345\210\207\345\210\206\346\223\215\344\275\234", Q_NULLPTR));
        displayLineSegAction->setText(QApplication::translate("MainWindow", "\346\237\245\347\234\213\346\211\200\351\200\211\347\272\277\346\256\265", Q_NULLPTR));
        switchLineSegAction->setText(QApplication::translate("MainWindow", "\347\272\277\346\256\265\345\210\207\346\215\242", Q_NULLPTR));
        performLineSegDelAction->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\345\210\240\351\231\244", Q_NULLPTR));
        polyPlanesAction_2->setText(QApplication::translate("MainWindow", "\345\271\263\351\235\242\345\244\232\350\276\271\345\275\242\345\214\226", Q_NULLPTR));
        openFile1_RegistrationAction->setText(QApplication::translate("MainWindow", "\351\205\215\345\207\206\347\202\271\344\272\221\346\226\207\344\273\2661", Q_NULLPTR));
        openFile2_RegistrationAction->setText(QApplication::translate("MainWindow", "\351\205\215\345\207\206\347\202\271\344\272\221\346\226\207\344\273\2662", Q_NULLPTR));
        registrationICPAction->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214ICP\351\205\215\345\207\206", Q_NULLPTR));
        action->setText(QApplication::translate("MainWindow", "\345\257\271\345\275\223\345\211\215\347\202\271\344\272\221\344\275\223\347\264\240\346\273\244\346\263\242", Q_NULLPTR));
        repairHolesAction->setText(QApplication::translate("MainWindow", "\346\217\220\345\217\226\345\255\224\346\264\236\350\276\271\347\274\230", Q_NULLPTR));
        openFile1RegistrationAction->setText(QApplication::translate("MainWindow", "\351\205\215\345\207\206\347\202\271\344\272\221\346\226\207\344\273\2661_target", Q_NULLPTR));
        openFile2RegistrationAction->setText(QApplication::translate("MainWindow", "\351\205\215\345\207\206\347\202\271\344\272\221\346\226\207\344\273\2662_source", Q_NULLPTR));
        registrationSACAction->setText(QApplication::translate("MainWindow", "SAC\347\262\227\351\205\215\345\207\206", Q_NULLPTR));
        rotatePointCloudAction->setText(QApplication::translate("MainWindow", "\346\227\213\350\275\254\347\202\271\344\272\221", Q_NULLPTR));
        registrationPlaneAction->setText(QApplication::translate("MainWindow", "\345\237\272\344\272\216\345\271\263\351\235\242\351\205\215\345\207\206", Q_NULLPTR));
        voxelGridFiltMergeCloudAction->setText(QApplication::translate("MainWindow", "\344\270\244\347\202\271\344\272\221\345\220\210\345\271\266\345\220\216\350\277\233\350\241\214\346\273\244\346\263\242", Q_NULLPTR));
        action_3->setText(QApplication::translate("MainWindow", "\344\270\244\347\202\271\344\272\221\345\220\210\345\271\266\346\273\244\346\263\242\345\271\266\344\277\235\345\255\230", Q_NULLPTR));
        cleanPointCloudAction->setText(QApplication::translate("MainWindow", "\346\270\205\345\261\217\347\202\271\344\272\221", Q_NULLPTR));
        removeNan1Action->setText(QApplication::translate("MainWindow", "\347\247\273\351\231\244NAN\347\202\271", Q_NULLPTR));
        voxelGridFilt1Action->setText(QApplication::translate("MainWindow", "\344\275\223\347\264\240\346\273\244\346\263\242", Q_NULLPTR));
        savePointCloudAction->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\345\275\223\345\211\215\347\202\271\344\272\221", Q_NULLPTR));
        repairHolesOFFAction->setText(QApplication::translate("MainWindow", "\350\277\233\350\241\214\345\255\224\346\264\236\344\277\256\345\244\215(\347\273\223\346\236\234\344\270\272pcd)", Q_NULLPTR));
        TriangularMeshingAction->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\347\275\221\346\240\274\345\214\226(\344\277\235\345\255\230\344\270\272.ply)", Q_NULLPTR));
        fileMenu->setTitle(QApplication::translate("MainWindow", "\346\226\207\344\273\266", Q_NULLPTR));
        renderPropertyMenu->setTitle(QApplication::translate("MainWindow", "\346\270\262\346\237\223\345\261\236\346\200\247", Q_NULLPTR));
        pointCloudPreProcessMenu->setTitle(QApplication::translate("MainWindow", "\347\202\271\344\272\221\351\242\204\345\244\204\347\220\206", Q_NULLPTR));
        param_set_menu->setTitle(QApplication::translate("MainWindow", "\345\217\202\346\225\260\350\256\276\347\275\256", Q_NULLPTR));
        plane_detect_menu->setTitle(QApplication::translate("MainWindow", "\345\271\263\351\235\242\346\217\220\345\217\226", Q_NULLPTR));
        normalProcessMenu->setTitle(QApplication::translate("MainWindow", "\346\263\225\345\220\221\351\207\217", Q_NULLPTR));
        regulateNormalMenu->setTitle(QApplication::translate("MainWindow", "\346\240\241\346\255\243\347\202\271\344\272\221\346\263\225\345\220\221\351\207\217", Q_NULLPTR));
        psMenu->setTitle(QApplication::translate("MainWindow", "\345\217\202\346\225\260\347\251\272\351\227\264", Q_NULLPTR));
        segmentMenu->setTitle(QApplication::translate("MainWindow", "\345\210\206\345\211\262", Q_NULLPTR));
        editPolyMenu->setTitle(QApplication::translate("MainWindow", "\345\244\232\350\276\271\345\275\242\347\274\226\350\276\221", Q_NULLPTR));
        delLineSegMenu->setTitle(QApplication::translate("MainWindow", "\347\272\277\346\256\265\345\210\240\351\231\244", Q_NULLPTR));
        registrationMenu->setTitle(QApplication::translate("MainWindow", "\351\205\215\345\207\206", Q_NULLPTR));
        repairHolesMenu->setTitle(QApplication::translate("MainWindow", "\345\255\224\346\264\236\344\277\256\345\244\215", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
