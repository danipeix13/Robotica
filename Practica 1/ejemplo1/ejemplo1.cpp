#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );

    stopped = false;
    time = 0;
    timerLimit = 500;
    timer = new QTimer(this);
    timer->start(timerLimit);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateDisplay()));
}

void ejemplo1::doButton()
{
    stopped = !stopped;
    stopped ? timer->stop() : timer->start(timerLimit);
}

void ejemplo1::updateDisplay() {
    lcdNumber->display(time++);
}




