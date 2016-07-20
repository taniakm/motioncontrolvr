#include "teleoperationgui.h"
#include <QtGui/QApplication>
#include "addtube.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	TeleoperationGUI w;
	w.show();
	return a.exec();
}
