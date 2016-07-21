#include "teleoperationgui.h"
#include "addtube.h"
#include "ui_addtube.h"
#include<QDebug>
//#include<QFile>
#include<QTextStream>
#include<QProcess>

TeleoperationGUI::TeleoperationGUI(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	// Setup filename
	tubeNum = 0;
}

TeleoperationGUI::~TeleoperationGUI()
{
	
}

void TeleoperationGUI::addTubeButtonPressed() {
	AddTube *dialog = new AddTube();
	dialog->exec();

	// Code to save entered parameters
	QString ODtemp = dialog->textOD->toPlainText();
	QString IDtemp = dialog->textID->toPlainText();
	QString kappatemp = dialog->textKappa->toPlainText();
	QString Lstemp = dialog->textLs->toPlainText();
	QString Lctemp = dialog->textLc->toPlainText();
	QString materialtemp = dialog->textMaterial->toPlainText();

	// Setting up file to save to
	QString filename = "tubeParameterFile";
	filename.append(QString::number(tubeNum));
	filename.append(".txt");
	qDebug() << filename;
	TeleoperationGUI::file.setFileName(filename);
	qDebug() << "filename set";
	qDebug() << tubeNum;
	// Write text to file
	if(TeleoperationGUI::file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QTextStream stream(&file);
		stream << "OD\t" << QString(ODtemp) << "\n";
		stream << "ID\t" << QString(IDtemp) << "\n";
		stream << "kappa\t" << QString(kappatemp) << "\n";
		stream << "Ls\t" << QString(Lstemp) << "\n";
		stream << "Lc\t" << QString(Lctemp) << "\n";
		stream << "material\t" << QString(materialtemp) << "\n";

		file.close();
		tubeNum = tubeNum + 1;

		qDebug() << "text written";
	} else {
		qDebug() << "couldn't open file";
	}
	
}

void TeleoperationGUI::doneButtonPressed() {
	QProcess *interfaceProcess = new QProcess(this);

	this->close();
}



