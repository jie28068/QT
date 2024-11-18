#ifndef LYRICS_H
#define LYRICS_H

#include <QWidget>
#include <QString>
#include <QMap>
#include <iostream>
#include <QFile>
#include <QDebug>
#include <QRegExp>
#include <QList>
#include <QRegularExpression>
#include <QRegularExpressionMatch>

using namespace std;
class Lyrics
{
private:
    QString lyricsPath;
    QMap<qint64, QString> listLyricsMap;

public:
    Lyrics(QString lyricsPath);
    Lyrics();
    bool readLyricsFile(QString lyricsPath);
    bool analysisLyricsFile(QString line);
    QMap<qint64, QString> getListLyricsMap() const;
};

#endif // LYRICS_H
