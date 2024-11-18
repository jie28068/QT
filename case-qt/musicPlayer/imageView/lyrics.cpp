#include "lyrics.h"

Lyrics::Lyrics(QString lyricsPath)
{
    this->lyricsPath = lyricsPath;
}
Lyrics::Lyrics()
{
}

QMap<qint64, QString> Lyrics::getListLyricsMap() const
{
    return listLyricsMap;
}
bool Lyrics::readLyricsFile(QString lyricsPath)
{
    listLyricsMap.clear();
    QFile file(lyricsPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        listLyricsMap.clear();
        return false;
    }
    QTextStream in(&file);
    in.setCodec("GB 2312");
    QString line;
    while (!in.atEnd())
    {
        line = in.readLine();
        analysisLyricsFile(line);
    }

    return true;
}

bool Lyrics::analysisLyricsFile(QString line)
{
    if (line == NULL || line.isEmpty())
    {
        qDebug() << "thie line is empty!";
        return false;
    }
    QRegExp timeRegExp("\\[(\\d+):(\\d+\\.\\d+)\\]");

    if (timeRegExp.indexIn(line) != -1)
    {
        qint64 totalTime = timeRegExp.cap(1).toInt() * 60000 + // 分钟
                           timeRegExp.cap(2).toFloat() * 1000; // 秒

        QString lyricText = line.mid(timeRegExp.matchedLength());
        listLyricsMap.insert(totalTime, lyricText);
    }
    return true;
}
