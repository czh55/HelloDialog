/********************************************************************************
** Form generated from reading UI file 'SetBGColorDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETBGCOLORDIALOG_H
#define UI_SETBGCOLORDIALOG_H

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
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SetBGColorDialog
{
public:
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLCDNumber *GlcdNumber;
    QLabel *label_3;
    QDialogButtonBox *buttonBox;
    QLabel *label_2;
    QLCDNumber *RlcdNumber;
    QLabel *label;
    QLCDNumber *BlcdNumber;
    QSpacerItem *horizontalSpacer;
    QSlider *bSlider;
    QSlider *gSlider;
    QSlider *rSlider;

    void setupUi(QDialog *SetBGColorDialog)
    {
        if (SetBGColorDialog->objectName().isEmpty())
            SetBGColorDialog->setObjectName(QStringLiteral("SetBGColorDialog"));
        SetBGColorDialog->resize(336, 181);
        gridLayoutWidget = new QWidget(SetBGColorDialog);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(30, 20, 271, 141));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        GlcdNumber = new QLCDNumber(gridLayoutWidget);
        GlcdNumber->setObjectName(QStringLiteral("GlcdNumber"));

        gridLayout->addWidget(GlcdNumber, 1, 3, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        buttonBox = new QDialogButtonBox(gridLayoutWidget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 5, 1, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        RlcdNumber = new QLCDNumber(gridLayoutWidget);
        RlcdNumber->setObjectName(QStringLiteral("RlcdNumber"));

        gridLayout->addWidget(RlcdNumber, 0, 3, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        BlcdNumber = new QLCDNumber(gridLayoutWidget);
        BlcdNumber->setObjectName(QStringLiteral("BlcdNumber"));

        gridLayout->addWidget(BlcdNumber, 2, 3, 1, 1);

        horizontalSpacer = new QSpacerItem(38, 18, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 5, 2, 1, 1);

        bSlider = new QSlider(gridLayoutWidget);
        bSlider->setObjectName(QStringLiteral("bSlider"));
        bSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(bSlider, 2, 1, 1, 2);

        gSlider = new QSlider(gridLayoutWidget);
        gSlider->setObjectName(QStringLiteral("gSlider"));
        gSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(gSlider, 1, 1, 1, 2);

        rSlider = new QSlider(gridLayoutWidget);
        rSlider->setObjectName(QStringLiteral("rSlider"));
        rSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(rSlider, 0, 1, 1, 2);


        retranslateUi(SetBGColorDialog);
        QObject::connect(rSlider, SIGNAL(valueChanged(int)), RlcdNumber, SLOT(display(int)));
        QObject::connect(gSlider, SIGNAL(valueChanged(int)), GlcdNumber, SLOT(display(int)));
        QObject::connect(bSlider, SIGNAL(valueChanged(int)), BlcdNumber, SLOT(display(int)));
        QObject::connect(buttonBox, SIGNAL(accepted()), SetBGColorDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SetBGColorDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(SetBGColorDialog);
    } // setupUi

    void retranslateUi(QDialog *SetBGColorDialog)
    {
        SetBGColorDialog->setWindowTitle(QApplication::translate("SetBGColorDialog", "Dialog", Q_NULLPTR));
        label_3->setText(QApplication::translate("SetBGColorDialog", "B", Q_NULLPTR));
        label_2->setText(QApplication::translate("SetBGColorDialog", "G", Q_NULLPTR));
        label->setText(QApplication::translate("SetBGColorDialog", "R", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SetBGColorDialog: public Ui_SetBGColorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETBGCOLORDIALOG_H
