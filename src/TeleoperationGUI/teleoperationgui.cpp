#include "teleoperationgui.h"
#include "addtube.h"
#include "ui_addtube.h"

TeleoperationGUI::TeleoperationGUI(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
}

TeleoperationGUI::~TeleoperationGUI()
{

}

void TeleoperationGUI::addTubeButtonPressed() {
	AddTube *dialog = new AddTube();
	dialog->exec();
}
