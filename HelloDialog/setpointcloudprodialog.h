#ifndef SETPOINTCLOUDPRODIALOG_H
#define SETPOINTCLOUDPRODIALOG_H

#include <QDialog>


namespace Ui {
class SetPointCloudProDialog;
}

class SetPointCloudProDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SetPointCloudProDialog(QWidget *parent = 0);
    ~SetPointCloudProDialog();

    void getData(int &R, int &G, int &B, int &size, QString &cloudID);

private:
    Ui::SetPointCloudProDialog *ui;
};

#endif // SETPOINTCLOUDPRODIALOG_H
