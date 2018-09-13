/********************************************************************************
** Form generated from reading UI file 'removepointclouddialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.6
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

QT_BEGIN_NAMESPACE

class Ui_RemovePointCloudDialog
{
public:
    QFormLayout *formLayout;
    QFormLayout *formLayout_2;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *RemovePointCloudDialog)
    {
        if (RemovePointCloudDialog->objectName().isEmpty())
            RemovePointCloudDialog->setObjectName(QStringLiteral("RemovePointCloudDialog"));
        RemovePointCloudDialog->resize(242, 78);
        formLayout = new QFormLayout(RemovePointCloudDialog);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QStringLiteral("formLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(RemovePointCloudDialog);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        lineEdit = new QLineEdit(RemovePointCloudDialog);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));

        horizontalLayout->addWidget(lineEdit);


        formLayout_2->setLayout(0, QFormLayout::LabelRole, horizontalLayout);

        buttonBox = new QDialogButtonBox(RemovePointCloudDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        formLayout_2->setWidget(1, QFormLayout::LabelRole, buttonBox);


        formLayout->setLayout(0, QFormLayout::LabelRole, formLayout_2);


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
