#ifndef ADDTUBE_H
#define ADDTUBE_H

#include <QDialog>
#include "ui_addtube.h"
#include "teleoperationgui.h"

class AddTube : public QDialog, public Ui::AddTube
{
	Q_OBJECT

public:
	AddTube(QWidget *parent = 0);
	~AddTube();

};

#endif // ADDTUBE_H
