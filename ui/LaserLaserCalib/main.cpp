#include "LaserLaserCalibMainwindow.h"
#include <QApplication>
#include <QTextCodec>
//qCC_db
#include <ccIncludeGL.h>
#include <ccTimer.h>
#include <ccNormalVectors.h>
#include <ccColorScalesManager.h>

using namespace std;
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	QApplication::addLibraryPath("./plugins");
	QTextCodec::setCodecForTr(QTextCodec::codecForLocale());
	//common data initialization
	ccTimer::Init();
	ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
	ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

	LaserLaserCalibMainwindow w;
	w.resize(800, 600);
	w.show();
	return app.exec();
}