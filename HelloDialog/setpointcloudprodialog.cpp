#include "setpointcloudprodialog.h"
#include "ui_setpointcloudprodialog.h"

SetPointCloudProDialog::SetPointCloudProDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetPointCloudProDialog)
{
    ui->setupUi(this);
}

SetPointCloudProDialog::~SetPointCloudProDialog()
{
    delete ui;
}

void SetPointCloudProDialog::getData(int &R, int &G, int &B, int &size, QString &cloudID)
{
    R = ui->rSlider->value();
    G = ui->gSlider->value();
    B = ui->bSlider->value();
    size = ui->sizeSlider->value();
    cloudID = ui->pointCloudID->text();
}
