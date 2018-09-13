/********************************************************************************
** Form generated from reading UI file 'setpointcloudprodialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.6
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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SetPointCloudProDialog
{
public:
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QLabel *label;
    QSlider *rSlider;
    QLCDNumber *RlcdNumber;
    QLabel *label_2;
    QSlider *gSlider;
    QLCDNumber *GlcdNumber;
    QLabel *label_3;
    QSlider *bSlider;
    QLCDNumber *BlcdNumber;
    QLabel *label_4;
    QSlider *sizeSlider;
    QLCDNumber *PointSizelcdNumber;
    QLabel *label_5;
    QLineEdit *pointCloudID;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *SetPointCloudProDialog)
    {
        if (SetPointCloudProDialog->objectName().isEmpty())
            SetPointCloudProDialog->setObjectName(QStringLiteral("SetPointCloudProDialog"));
        SetPointCloudProDialog->resize(409, 175);
        layoutWidget = new QWidget(SetPointCloudProDialog);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(12, 20, 387, 141));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        rSlider = new QSlider(layoutWidget);
        rSlider->setObjectName(QStringLiteral("rSlider"));
        rSlider->setMaximum(255);
        rSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(rSlider, 0, 1, 1, 2);

        RlcdNumber = new QLCDNumber(layoutWidget);
        RlcdNumber->setObjectName(QStringLiteral("RlcdNumber"));
        RlcdNumber->setFrameShape(QFrame::Box);

        gridLayout->addWidget(RlcdNumber, 0, 3, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        gSlider = new QSlider(layoutWidget);
        gSlider->setObjectName(QStringLiteral("gSlider"));
        gSlider->setMaximum(255);
        gSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(gSlider, 1, 1, 1, 2);

        GlcdNumber = new QLCDNumber(layoutWidget);
        GlcdNumber->setObjectName(QStringLiteral("GlcdNumber"));

        gridLayout->addWidget(GlcdNumber, 1, 3, 1, 1);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        bSlider = new QSlider(layoutWidget);
        bSlider->setObjectName(QStringLiteral("bSlider"));
        bSlider->setMaximum(255);
        bSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(bSlider, 2, 1, 1, 2);

        BlcdNumber = new QLCDNumber(layoutWidget);
        BlcdNumber->setObjectName(QStringLiteral("BlcdNumber"));

        gridLayout->addWidget(BlcdNumber, 2, 3, 1, 1);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 3, 0, 1, 1);

        sizeSlider = new QSlider(layoutWidget);
        sizeSlider->setObjectName(QStringLiteral("sizeSlider"));
        sizeSlider->setMinimum(1);
        sizeSlider->setMaximum(15);
        sizeSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(sizeSlider, 3, 1, 1, 2);

        PointSizelcdNumber = new QLCDNumber(layoutWidget);
        PointSizelcdNumber->setObjectName(QStringLiteral("PointSizelcdNumber"));

        gridLayout->addWidget(PointSizelcdNumber, 3, 3, 1, 1);

        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 4, 0, 1, 1);

        pointCloudID = new QLineEdit(layoutWidget);
        pointCloudID->setObjectName(QStringLiteral("pointCloudID"));

        gridLayout->addWidget(pointCloudID, 4, 1, 1, 1);

        buttonBox = new QDialogButtonBox(layoutWidget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 4, 2, 1, 2);

#ifndef QT_NO_SHORTCUT
        label->setBuddy(rSlider);
        label_2->setBuddy(gSlider);
        label_3->setBuddy(bSlider);
        label_4->setBuddy(sizeSlider);
#endif // QT_NO_SHORTCUT

        retranslateUi(SetPointCloudProDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), SetPointCloudProDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SetPointCloudProDialog, SLOT(reject()));
        QObject::connect(rSlider, SIGNAL(valueChanged(int)), RlcdNumber, SLOT(display(int)));
        QObject::connect(gSlider, SIGNAL(valueChanged(int)), GlcdNumber, SLOT(display(int)));
        QObject::connect(bSlider, SIGNAL(valueChanged(int)), BlcdNumber, SLOT(display(int)));
        QObject::connect(sizeSlider, SIGNAL(valueChanged(int)), PointSizelcdNumber, SLOT(display(int)));

        QMetaObject::connectSlotsByName(SetPointCloudProDialog);
    } // setupUi

    void retranslateUi(QDialog *SetPointCloudProDialog)
    {
        SetPointCloudProDialog->setWindowTitle(QApplication::translate("SetPointCloudProDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("SetPointCloudProDialog", "Red Channel:", Q_NULLPTR));
        label_2->setText(QApplication::translate("SetPointCloudProDialog", "Green Channel:", Q_NULLPTR));
        label_3->setText(QApplication::translate("SetPointCloudProDialog", "Blue Channel:", Q_NULLPTR));
        label_4->setText(QApplication::translate("SetPointCloudProDialog", "PointSize:", Q_NULLPTR));
        label_5->setText(QApplication::translate("SetPointCloudProDialog", "PointCloudID:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SetPointCloudProDialog: public Ui_SetPointCloudProDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETPOINTCLOUDPRODIALOG_H
