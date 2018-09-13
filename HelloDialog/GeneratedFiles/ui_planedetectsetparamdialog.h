/********************************************************************************
** Form generated from reading UI file 'planedetectsetparamdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLANEDETECTSETPARAMDIALOG_H
#define UI_PLANEDETECTSETPARAMDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTableView>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_PlaneDetectSetParamDialog
{
public:
    QFormLayout *formLayout;
    QVBoxLayout *verticalLayout;
    QTableView *tableView;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *PlaneDetectSetParamDialog)
    {
        if (PlaneDetectSetParamDialog->objectName().isEmpty())
            PlaneDetectSetParamDialog->setObjectName(QStringLiteral("PlaneDetectSetParamDialog"));
        PlaneDetectSetParamDialog->resize(723, 549);
        formLayout = new QFormLayout(PlaneDetectSetParamDialog);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        tableView = new QTableView(PlaneDetectSetParamDialog);
        tableView->setObjectName(QStringLiteral("tableView"));
        tableView->setMinimumSize(QSize(0, 500));

        verticalLayout->addWidget(tableView);


        formLayout->setLayout(0, QFormLayout::SpanningRole, verticalLayout);

        buttonBox = new QDialogButtonBox(PlaneDetectSetParamDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(buttonBox->sizePolicy().hasHeightForWidth());
        buttonBox->setSizePolicy(sizePolicy);
        buttonBox->setLayoutDirection(Qt::LeftToRight);
        buttonBox->setAutoFillBackground(false);
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);

        formLayout->setWidget(1, QFormLayout::LabelRole, buttonBox);


        retranslateUi(PlaneDetectSetParamDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), PlaneDetectSetParamDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), PlaneDetectSetParamDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(PlaneDetectSetParamDialog);
    } // setupUi

    void retranslateUi(QDialog *PlaneDetectSetParamDialog)
    {
        PlaneDetectSetParamDialog->setWindowTitle(QApplication::translate("PlaneDetectSetParamDialog", "\345\271\263\351\235\242\346\217\220\345\217\226\345\217\202\346\225\260\350\256\276\347\275\256", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PlaneDetectSetParamDialog: public Ui_PlaneDetectSetParamDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLANEDETECTSETPARAMDIALOG_H
