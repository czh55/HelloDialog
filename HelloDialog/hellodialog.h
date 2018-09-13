#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_hellodialog.h"

class HelloDialog : public QMainWindow
{
	Q_OBJECT

public:
	HelloDialog(QWidget *parent = Q_NULLPTR);

private:
	Ui::HelloDialog ui;
};
