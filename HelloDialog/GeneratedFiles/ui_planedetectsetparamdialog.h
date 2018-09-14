/********************************************************************************
** Form generated from reading UI file 'PlaneDetectSetParamDialog.ui'
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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTableView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PlaneDetectSetParamDialog
{
public:
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QTableView *tableView;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *PlaneDetectSetParamDialog)
    {
        if (PlaneDetectSetParamDialog->objectName().isEmpty())
            PlaneDetectSetParamDialog->setObjectName(QStringLiteral("PlaneDetectSetParamDialog"));
        PlaneDetectSetParamDialog->resize(649, 431);
        verticalLayoutWidget = new QWidget(PlaneDetectSetParamDialog);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(0, 0, 651, 391));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        tableView = new QTableView(verticalLayoutWidget);
        tableView->setObjectName(QStringLiteral("tableView"));

        verticalLayout->addWidget(tableView);

        buttonBox = new QDialogButtonBox(PlaneDetectSetParamDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(10, 400, 156, 23));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        retranslateUi(PlaneDetectSetParamDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), PlaneDetectSetParamDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), PlaneDetectSetParamDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(PlaneDetectSetParamDialog);
    } // setupUi

    void retranslateUi(QDialog *PlaneDetectSetParamDialog)
    {
        PlaneDetectSetParamDialog->setWindowTitle(QApplication::translate("PlaneDetectSetParamDialog", "Dialog", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PlaneDetectSetParamDialog: public Ui_PlaneDetectSetParamDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLANEDETECTSETPARAMDIALOG_H
