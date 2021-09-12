#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        /*
         * Constructor method
         */
        ejemplo1();

    public slots:
        /*
         * Starts and stops the timer
         */
        void doButton();

        /*
         * Shows a new value on the display
         */
        void updateDisplay();

    private:
        bool stopped;
        int time, timerLimit;
};

#endif // ejemplo1_H
