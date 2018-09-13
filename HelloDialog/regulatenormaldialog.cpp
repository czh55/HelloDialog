#include "regulatenormaldialog.h"
#include "ui_regulatenormaldialog.h"

RegulateNormalDialog::RegulateNormalDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RegulateNormalDialog)
{
    ui->setupUi(this);
    is_norm_direction_valid = true;
}

RegulateNormalDialog::~RegulateNormalDialog()
{
    delete ui;
}

void RegulateNormalDialog::on_buttonBox_accepted()
{
    is_norm_direction_valid = true;
}

void RegulateNormalDialog::on_buttonBox_rejected()
{
    is_norm_direction_valid = false;
}
