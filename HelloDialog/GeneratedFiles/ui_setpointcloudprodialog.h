/********************************************************************************
** Form generated from reading UI file 'SetPointCloudProDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETPOINTCLOUDPRODIALOG_H
#define UI_SETPOINTCLOUDPRODIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SetPointCloudProDialog
{
public:
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QSlider *rSlider;
    QSlider *bSlider;
    QLabel *label;
    QLabel *label_2;
    QSlider *sizeSlider;
    QLabel *label_4;
    QSlider *gSlider;
    QLCDNumber *BlcdNumber;
    QLCDNumber *PointSizelcdNumber;
    QLabel *label_3;
    QLCDNumber *RlcdNumber;
    QLCDNumber *GlcdNumber;
    QHBoxLayout *horizontalLayout;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *SetPointCloudProDialog)
    {
        if (SetPointCloudProDialog->objectName().isEmpty())
            SetPointCloudProDialog->setObjectName(QStringLiteral("SetPointCloudProDialog"));
        SetPointCloudProDialog->resize(399, 188);
        gridLayoutWidget = new QWidget(SetPointCloudProDialog);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(30, 20, 341, 151));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        rSlider = new QSlider(gridLayoutWidget);
        rSlider->setObjectName(QStringLiteral("rSlider"));
        rSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(rSlider, 0, 1, 1, 1);

        bSlider = new QSlider(gridLayoutWidget);
        bSlider->setObjectName(QStringLiteral("bSlider"));
        bSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(bSlider, 4, 1, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        sizeSlider = new QSlider(gridLayoutWidget);
        sizeSlider->setObjectName(QStringLiteral("sizeSlider"));
        sizeSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(sizeSlider, 6, 1, 1, 1);

        label_4 = new QLabel(gridLayoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 6, 0, 1, 1);

        gSlider = new QSlider(gridLayoutWidget);
        gSlider->setObjectName(QStringLiteral("gSlider"));
        gSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(gSlider, 2, 1, 1, 1);

        BlcdNumber = new QLCDNumber(gridLayoutWidget);
        BlcdNumber->setObjectName(QStringLiteral("BlcdNumber"));

        gridLayout->addWidget(BlcdNumber, 4, 2, 1, 1);

        PointSizelcdNumber = new QLCDNumber(gridLayoutWidget);
        PointSizelcdNumber->setObjectName(QStringLiteral("PointSizelcdNumber"));

        gridLayout->addWidget(PointSizelcdNumber, 6, 2, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 4, 0, 1, 1);

        RlcdNumber = new QLCDNumber(gridLayoutWidget);
        RlcdNumber->setObjectName(QStringLiteral("RlcdNumber"));

        gridLayout->addWidget(RlcdNumber, 0, 2, 1, 1);

        GlcdNumber = new QLCDNumber(gridLayoutWidget);
        GlcdNumber->setObjectName(QStringLiteral("GlcdNumber"));

        gridLayout->addWidget(GlcdNumber, 2, 2, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        buttonBox = new QDialogButtonBox(gridLayoutWidget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setLayoutDirection(Qt::RightToLeft);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        horizontalLayout->addWidget(buttonBox);


        gridLayout->addLayout(horizontalLayout, 9, 1, 1, 2);


        retranslateUi(SetPointCloudProDialog);
        QObject::connect(rSlider, SIGNAL(valueChanged(int)), RlcdNumber, SLOT(display(int)));
        QObject::connect(gSlider, SIGNAL(valueChanged(int)), GlcdNumber, SLOT(display(int)));
        QObject::connect(bSlider, SIGNAL(valueChanged(int)), BlcdNumber, SLOT(display(int)));
        QObject::connect(buttonBox, SIGNAL(accepted()), SetPointCloudProDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SetPointCloudProDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(SetPointCloudProDialog);
    } // setupUi

    void retranslateUi(QDialog *SetPointCloudProDialog)
    {
        SetPointCloudProDialog->setWindowTitle(QApplication::translate("SetPointCloudProDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("SetPointCloudProDialog", "Red Channel:", Q_NULLPTR));
        label_2->setText(QApplication::translate("SetPointCloudProDialog", "Green Channel:", Q_NULLPTR));
        label_4->setText(QApplication::translate("SetPointCloudProDialog", "PointSize:", Q_NULLPTR));
        label_3->setText(QApplication::translate("SetPointCloudProDialog", "Blue Channel:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SetPointCloudProDialog: public Ui_SetPointCloudProDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETPOINTCLOUDPRODIALOG_H
