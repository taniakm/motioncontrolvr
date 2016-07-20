#ifndef TELEOPERATIONGUI_H
#define TELEOPERATIONGUI_H

#include <QtGui/QMainWindow>
#include "ui_teleoperationgui.h"
#include<QFile>

class TeleoperationGUI : public QMainWindow
{
	Q_OBJECT

public:
	TeleoperationGUI(QWidget *parent = 0, Qt::WFlags flags = 0);
	~TeleoperationGUI();

	QFile file;
	int tubeNum;
	

	private slots:
		void addTubeButtonPressed();

private:
	Ui::TeleoperationGUIClass ui;
	//int tubeNum;

};

#endif // TELEOPERATIONGUI_H
