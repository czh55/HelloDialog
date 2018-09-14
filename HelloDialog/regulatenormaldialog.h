#ifndef REGULATENORMALDIALOG_H
#define REGULATENORMALDIALOG_H

#include <QDialog>
//#include "PlaneDetect.h"

namespace Ui {
class RegulateNormalDialog;
}

class RegulateNormalDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RegulateNormalDialog(QWidget *parent = 0);
    ~RegulateNormalDialog();
    bool is_norm_direction_valid;
private:
    Ui::RegulateNormalDialog *ui;

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
};

#endif // REGULATENORMALDIALOG_H
