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
		stream << "OD:" << QString(ODtemp) << ",";
		stream << "ID:" << QString(IDtemp) << ",";
		stream << "kappa:" << QString(kappatemp) << ",";
		stream << "Ls:" << QString(Lstemp) << ",";
		stream << "Lc:" << QString(Lctemp) << ",";
		stream << "material:" << QString(materialtemp) << ";";

		file.close();
		tubeNum = tubeNum + 1;

		qDebug() << "text written";
	} else {
		qDebug() << "couldn't open file";
	}
	
}

void TeleoperationGUI::doneButtonPressed() {
	// Save number of tubes and write to file
	QString numTubesTemp = ui.textNumTubes->toPlainText();
	QString filename = "numTubesFile.txt";
	TeleoperationGUI::file.setFileName(filename);
	if(TeleoperationGUI::file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QTextStream stream(&file);
		stream << QString(numTubesTemp) << "\n";
		file.close();
	}
	// Write 0 to useDefaultSet.txt
	filename = "useDefaultSet.txt";
	TeleoperationGUI::file.setFileName(filename);
	QString useDefaultSetBool = QString::number(0);
	if(TeleoperationGUI::file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QTextStream stream(&file);
		stream << QString(useDefaultSetBool) << "\n";
		file.close();
	}

	// close GUI
	this->close();
}

void TeleoperationGUI::defaultButtonPressed() {
	// Save default number of tubes (3)
	QString numTubesTemp = QString::number(3);
	QString filename = "numTubesFile.txt";
	TeleoperationGUI::file.setFileName(filename);
	if(TeleoperationGUI::file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QTextStream stream(&file);
		stream << QString(numTubesTemp) << "\n";
		file.close();
	}
	// Write 1 to useDefaultSet.txt
	filename = "useDefaultSet.txt";
	TeleoperationGUI::file.setFileName(filename);
	QString useDefaultSetBool = QString::number(1);
	if(TeleoperationGUI::file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QTextStream stream(&file);
		stream << QString(useDefaultSetBool) << "\n";
		file.close();
	}

	// close GUI
	this->close();
}



