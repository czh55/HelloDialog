#include "removepointclouddialog.h"
#include "ui_removepointclouddialog.h"

RemovePointCloudDialog::RemovePointCloudDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RemovePointCloudDialog)
{
    ui->setupUi(this);
}

RemovePointCloudDialog::~RemovePointCloudDialog()
{
    delete ui;
}

void RemovePointCloudDialog::on_buttonBox_accepted()
{
    cloudID = ui->lineEdit->text();
}