#ifndef TELEOPERATIONGUI_H
#define TELEOPERATIONGUI_H

#include <QtGui/QMainWindow>
#include "ui_teleoperationgui.h"

class TeleoperationGUI : public QMainWindow
{
	Q_OBJECT

public:
	TeleoperationGUI(QWidget *parent = 0, Qt::WFlags flags = 0);
	~TeleoperationGUI();

	private slots:
		void addTubeButtonPressed();

private:
	Ui::TeleoperationGUIClass ui;

};

#endif // TELEOPERATIONGUI_H
