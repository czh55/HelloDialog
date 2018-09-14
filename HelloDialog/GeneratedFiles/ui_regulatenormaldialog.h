/********************************************************************************
** Form generated from reading UI file 'RegulateNormalDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REGULATENORMALDIALOG_H
#define UI_REGULATENORMALDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RegulateNormalDialog
{
public:
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *RegulateNormalDialog)
    {
        if (RegulateNormalDialog->objectName().isEmpty())
            RegulateNormalDialog->setObjectName(QStringLiteral("RegulateNormalDialog"));
        RegulateNormalDialog->resize(439, 119);
        verticalLayoutWidget = new QWidget(RegulateNormalDialog);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 20, 422, 80));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(verticalLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout->addWidget(label);

        buttonBox = new QDialogButtonBox(verticalLayoutWidget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(RegulateNormalDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), RegulateNormalDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), RegulateNormalDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(RegulateNormalDialog);
    } // setupUi

    void retranslateUi(QDialog *RegulateNormalDialog)
    {
        RegulateNormalDialog->setWindowTitle(QApplication::translate("RegulateNormalDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("RegulateNormalDialog", "\351\200\211\346\213\251\347\202\271\344\272\221\344\270\255\347\232\204\347\202\271\357\274\214\347\234\213\345\205\266\346\263\225\345\220\221\351\207\217\346\230\257\345\220\246\346\214\207\345\220\221\347\211\251\344\275\223\345\244\226\351\203\250\357\274\210\346\230\257\357\274\214\347\202\271\345\207\273ok\357\274\233\345\220\246\347\202\271\345\207\273cancel\357\274\211", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class RegulateNormalDialog: public Ui_RegulateNormalDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REGULATENORMALDIALOG_H
