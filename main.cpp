#include <QApplication>
#include "siftmatch.h"
#include <QTextCodec>

int main(int argc, char *argv[])
{
    QTextCodec::setCodecForTr(QTextCodec::codecForLocale());

    QApplication a(argc, argv);
    SiftMatch w;
    w.show();
    
    return a.exec();
}
