#include "SetPointCloudProDialog.h"
#include "ui_SetPointCloudProDialog.h"

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

void SetPointCloudProDialog::getData(int &R, int &G, int &B, int &size)
{
    R = ui->rSlider->value();
    G = ui->gSlider->value();
    B = ui->bSlider->value();
    size = ui->sizeSlider->value();
}
