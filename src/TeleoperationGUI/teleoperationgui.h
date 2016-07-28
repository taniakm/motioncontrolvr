#ifndef TELEOPERATIONGUI_H
#define TELEOPERATIONGUI_H

#include <QtGui/QMainWindow>
#include "ui_teleoperationgui.h"
#include<QFile>
#include "addtube.h"

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
		void doneButtonPressed();
		void defaultButtonPressed();

private:
	Ui::TeleoperationGUIClass ui;

};

#endif // TELEOPERATIONGUI_H
