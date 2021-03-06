#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <chrono>
#include "timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        /*
         * Constructor method that starts the user interface . It links the button with its event and the timer with the function "cuenta"
         */
        ejemplo1();

        /*
        * Destructor method
        */
        virtual ~ejemplo1();

    public slots:
        /*
         * Changes the chrono's behaviour. If the chrono is running it will stop and viceversa.
         */
	    void doButton();

        /*
        * Swaps chronometer's behaviour between upwards and downwards (upwards as default).
        */
        void invert();

    private:
        Timer mytimer, mytimerLong;
        int cont = 0, trick = 5, timerPeriod = 500;
        bool inverted = false;

        /*
        * Prints the chrono's number using the UI.
        */
        void cuenta();

};

#endif // ejemplo1_H
