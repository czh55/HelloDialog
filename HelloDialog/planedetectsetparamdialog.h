#ifndef PLANEDETECTSETPARAMDIALOG_H
#define PLANEDETECTSETPARAMDIALOG_H

#include <QDialog>
#include <QStandardItemModel>
#include <QString>
#include <QSettings>



namespace Ui {
class PlaneDetectSetParamDialog;
}

class PlaneDetectSetParamDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PlaneDetectSetParamDialog(QWidget *parent = 0);
    ~PlaneDetectSetParamDialog();

private:
    Ui::PlaneDetectSetParamDialog *ui;
    QStandardItemModel *model;
    QSettings *configIniRead;

    void initTable();
    void setTableItem(int row, QString str1, QString str2);

private slots:
    void on_buttonBox_accepted();
};

#endif // PLANEDETECTSETPARAMDIALOG_H
