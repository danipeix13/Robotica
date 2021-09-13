#include "ejemplo1.h"


ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(invertButton, SIGNAL(clicked()), this, SLOT(invert()));
	
	mytimer.connect(std::bind(&ejemplo1::cuenta, this));//Connect the thre
    mytimer.start(timerPeriod);
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
	static bool stopped = false;
	stopped = !stopped;
	if(stopped)
		mytimer.stop();
	else
		mytimer.start(timerPeriod);
	qDebug() << "click on button";
}

void ejemplo1::invert()
{
    inverted = !inverted;
}

void ejemplo1::cuenta()
{
    lcdNumber->display((!inverted) ? ++cont : (cont > 0) ? --cont : 0);
	trick++;
}

