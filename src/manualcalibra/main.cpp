#include "qt/widget.h"

#include <QApplication>

#include <QObject>

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    Widget w(argc, argv);
    w.show();

    return a.exec();
}