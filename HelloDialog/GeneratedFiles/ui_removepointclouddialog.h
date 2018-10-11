/********************************************************************************
** Form generated from reading UI file 'RemovePointCloudDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REMOVEPOINTCLOUDDIALOG_H
#define UI_REMOVEPOINTCLOUDDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RemovePointCloudDialog
{
public:
    QWidget *formLayoutWidget;
    QFormLayout *formLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *RemovePointCloudDialog)
    {
        if (RemovePointCloudDialog->objectName().isEmpty())
            RemovePointCloudDialog->setObjectName(QStringLiteral("RemovePointCloudDialog"));
        RemovePointCloudDialog->resize(400, 300);
        formLayoutWidget = new QWidget(RemovePointCloudDialog);
        formLayoutWidget->setObjectName(QStringLiteral("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(80, 110, 221, 61));
        formLayout = new QFormLayout(formLayoutWidget);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(formLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        lineEdit = new QLineEdit(formLayoutWidget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));

        horizontalLayout->addWidget(lineEdit);


        formLayout->setLayout(0, QFormLayout::LabelRole, horizontalLayout);

        buttonBox = new QDialogButtonBox(formLayoutWidget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        formLayout->setWidget(1, QFormLayout::LabelRole, buttonBox);


        retranslateUi(RemovePointCloudDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), RemovePointCloudDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), RemovePointCloudDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(RemovePointCloudDialog);
    } // setupUi

    void retranslateUi(QDialog *RemovePointCloudDialog)
    {
        RemovePointCloudDialog->setWindowTitle(QApplication::translate("RemovePointCloudDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("RemovePointCloudDialog", "\350\276\223\345\205\245\347\202\271\344\272\221ID\357\274\232", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class RemovePointCloudDialog: public Ui_RemovePointCloudDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REMOVEPOINTCLOUDDIALOG_H
