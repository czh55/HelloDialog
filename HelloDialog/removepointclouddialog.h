#ifndef REMOVEPOINTCLOUDDIALOG_H
#define REMOVEPOINTCLOUDDIALOG_H



#include <QDialog>

namespace Ui {
class RemovePointCloudDialog;
}

class RemovePointCloudDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RemovePointCloudDialog(QWidget *parent = 0);
    ~RemovePointCloudDialog();

    QString cloudID;

private:
    Ui::RemovePointCloudDialog *ui;
private slots:
    void on_buttonBox_accepted();
};

#endif // REMOVEPOINTCLOUDDIALOG_H
