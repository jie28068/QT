#include <QApplication>

#include "myXml.h"

#include <QFile>
#include <QTextStream>
#include <QDomDocument>

QString filepath2 = "D:\\GitProject\\my-case\\QT\\read_write_xml\\my2.xml";
void readXml()
{
    QFile file(filepath2);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Error: Cannot read the file!";
        return;
    }

    QXmlStreamReader xmlReader;
    xmlReader.setDevice(&file);

    while (!xmlReader.atEnd() && !xmlReader.hasError())
    {
        QXmlStreamReader::TokenType token = xmlReader.readNext();
        if (token == QXmlStreamReader::StartElement)
        {
            if (xmlReader.name() == "elementName")
            {
                QString text = xmlReader.readElementText();
                qDebug() << "Element found: " << text;
            }
        }
    }

    if (xmlReader.hasError())
    {
        qDebug() << "Error: " << xmlReader.errorString();
    }

    file.close();
}

void writeXml()
{
    QFile file(filepath2);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Error: Cannot write to the file!";
        return;
    }

    QXmlStreamWriter xmlWriter(&file);
    xmlWriter.setAutoFormatting(true);
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("root");

    xmlWriter.writeStartElement("elementName");
    xmlWriter.writeCharacters("Element Content");
    xmlWriter.writeEndElement();

    xmlWriter.writeEndElement();
    xmlWriter.writeEndDocument();

    file.close();
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    readXml();
    writeXml();
    BoardSettingWidget *browser = new BoardSettingWidget();
    if (browser->exec() != QDialog::Accepted)
    {
        browser->wirteCurrentXmlContent();
    }

    return app.exec();
}