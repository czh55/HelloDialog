#include "setbgcolordialog.h"
#include "ui_setbgcolordialog.h"

SetBGColorDialog::SetBGColorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetBGColorDialog)
{
    ui->setupUi(this);

    ui->rSlider->setMinimum(0);
    ui->rSlider->setMaximum(255);
    ui->gSlider->setMinimum(0);
    ui->gSlider->setMaximum(255);
    ui->bSlider->setMinimum(0);
    ui->bSlider->setMaximum(255);
}

SetBGColorDialog::~SetBGColorDialog()
{
    delete ui;
}

void SetBGColorDialog::getRGB(int &R, int &G, int &B)
{
    R = ui->rSlider->value();
    G = ui->gSlider->value();
    B = ui->bSlider->value();
}
