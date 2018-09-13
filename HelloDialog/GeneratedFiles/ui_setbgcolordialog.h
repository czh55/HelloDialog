/********************************************************************************
** Form generated from reading UI file 'setbgcolordialog.ui'
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
    QWidget *widget;
    QGridLayout *gridLayout;
    QLabel *label;
    QLCDNumber *RlcdNumber;
    QSlider *rSlider;
    QLabel *label_2;
    QLCDNumber *GlcdNumber;
    QSlider *gSlider;
    QLabel *label_3;
    QLCDNumber *BlcdNumber;
    QSlider *bSlider;
    QDialogButtonBox *buttonBox;
    QSpacerItem *horizontalSpacer;

    void setupUi(QDialog *SetBGColorDialog)
    {
        if (SetBGColorDialog->objectName().isEmpty())
            SetBGColorDialog->setObjectName(QStringLiteral("SetBGColorDialog"));
        SetBGColorDialog->resize(333, 190);
        widget = new QWidget(SetBGColorDialog);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(11, 31, 292, 118));
        gridLayout = new QGridLayout(widget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 0, 0, 2, 2);

        RlcdNumber = new QLCDNumber(widget);
        RlcdNumber->setObjectName(QStringLiteral("RlcdNumber"));

        gridLayout->addWidget(RlcdNumber, 0, 5, 2, 1);

        rSlider = new QSlider(widget);
        rSlider->setObjectName(QStringLiteral("rSlider"));
        rSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(rSlider, 1, 1, 1, 4);

        label_2 = new QLabel(widget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 2, 0, 2, 2);

        GlcdNumber = new QLCDNumber(widget);
        GlcdNumber->setObjectName(QStringLiteral("GlcdNumber"));

        gridLayout->addWidget(GlcdNumber, 2, 5, 2, 1);

        gSlider = new QSlider(widget);
        gSlider->setObjectName(QStringLiteral("gSlider"));
        gSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(gSlider, 3, 1, 1, 4);

        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_3, 4, 0, 2, 2);

        BlcdNumber = new QLCDNumber(widget);
        BlcdNumber->setObjectName(QStringLiteral("BlcdNumber"));

        gridLayout->addWidget(BlcdNumber, 4, 5, 2, 1);

        bSlider = new QSlider(widget);
        bSlider->setObjectName(QStringLiteral("bSlider"));
        bSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(bSlider, 5, 1, 1, 4);

        buttonBox = new QDialogButtonBox(widget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 6, 2, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 6, 3, 1, 1);


        retranslateUi(SetBGColorDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), SetBGColorDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SetBGColorDialog, SLOT(reject()));
        QObject::connect(rSlider, SIGNAL(valueChanged(int)), RlcdNumber, SLOT(display(int)));
        QObject::connect(gSlider, SIGNAL(valueChanged(int)), GlcdNumber, SLOT(display(int)));
        QObject::connect(bSlider, SIGNAL(valueChanged(int)), BlcdNumber, SLOT(display(int)));

        QMetaObject::connectSlotsByName(SetBGColorDialog);
    } // setupUi

    void retranslateUi(QDialog *SetBGColorDialog)
    {
        SetBGColorDialog->setWindowTitle(QApplication::translate("SetBGColorDialog", "\350\256\276\347\275\256\350\203\214\346\231\257\351\242\234\350\211\262", Q_NULLPTR));
        label->setText(QApplication::translate("SetBGColorDialog", "R", Q_NULLPTR));
        label_2->setText(QApplication::translate("SetBGColorDialog", "G", Q_NULLPTR));
        label_3->setText(QApplication::translate("SetBGColorDialog", "B", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SetBGColorDialog: public Ui_SetBGColorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETBGCOLORDIALOG_H
