[TOC]

QT 使用QXmlStreamReader/QXmlStreamWriter和QDomDocument俩种方式读写XML文件
## 效果图

- 我们可以直接将控件或其他配置的值写到xml文件中，这样下一次打开时，直接读取xml中的值进行初始化窗口。
## 使用`QDomElement`读写
`QDomDocument` 是 Qt 框架中用于处理 XML 的类，它提供了一个以 DOM（文档对象模型）方式解析和创建 XML 文档的接口。DOM 是 W3C 推荐的处理 XML 的标准编程接口，它将 XML 文档转换为一个树形结构，可以对其进行遍历和修改。
以下是使用 `QDomDocument` 读写 XML 的基本步骤：
### 读取 XML 文档
1. **创建 QDomDocument 对象**：
   ```cpp
   QDomDocument doc;
   ```
2. **加载 XML 文档**：
   有几种方式可以加载 XML 文档，比如从文件、字符串等：
   ```cpp
   QFile file("example.xml");
   if (!file.open(QIODevice::ReadOnly))
       return;
   if (!doc.setContent(&file)) {
       file.close();
       return;
   }
   file.close();
   ```
3. **访问文档元素**：
   加载完成后，可以访问根元素，然后遍历或查询其他元素：
   ```cpp
   QDomElement root = doc.documentElement();
   QDomNode child = root.firstChild();
   while(!child.isNull()) {
       QDomElement element = child.toElement();
       if(!element.isNull()) {
           // 处理元素
       }
       child = child.nextSibling();
   }
   ```
4. **读取元素数据**：
   可以读取元素的属性和文本内容：
   ```cpp
   QString text = element.text();
   QString attribute = element.attribute("attributeName");
   ```
### 创建或修改 XML 文档
1. **创建 QDomDocument 对象**：
   如果是从头创建 XML，首先创建一个空的 `QDomDocument` 对象。
2. **创建和设置根元素**：
   ```cpp
   QDomElement root = doc.createElement("rootElement");
   doc.appendChild(root);
   ```
3. **添加子元素和属性**：
   ```cpp
   QDomElement child = doc.createElement("childElement");
   root.appendChild(child);
   QDomText text = doc.createTextNode("Some text");
   child.appendChild(text);
   child.setAttribute("attributeName", "attributeValue");
   ```
4. **保存 XML 文档**：
   可以将 `QDomDocument` 对象的内容保存到文件或字符串：
   ```cpp
   QFile file("output.xml");
   if (!file.open(QIODevice::WriteOnly))
       return;
   QTextStream out(&file);
   doc.save(out, 4);  // 参数 4 表示缩进为 4 个空格
   file.close();
   ```
在使用 `QDomDocument` 时，需要注意错误处理和异常安全。例如，在尝试打开文件或解析 XML 时，应该检查可能出现的错误，并做出相应的处理。
以上是 `QDomDocument` 的基本使用方法。在实际应用中，可能还需要更复杂的操作，比如 XPath 查询、处理命名空间等。Qt 提供了丰富的 API 来支持这些高级功能。
`QXmlStreamReader` 和 `QXmlStreamWriter` 是 Qt 框架中用于处理 XML 文档的类。它们提供了一种快速、只读的 API 来解析 XML，以及一个用于写入 XML 文档的 API。这两个类都是基于流（Stream）的，这意味着它们可以处理大型的 XML 文档，而不需要将整个文档加载到内存中。

## 使用`QXmlStreamReader`和`QXmlStreamWriter`读写
### QXmlStreamReader
`QXmlStreamReader` 用于读取和解析 XML 文档。它逐个读取 XML 元素、属性和字符数据，并将其转换为不同的类型，如StartElement、EndElement、Characters等。使用 `QXmlStreamReader` 可以轻松地遍历 XML 文档并提取所需的信息。
以下是使用 `QXmlStreamReader` 读取 XML 文档的一个简单示例：
```cpp
#include <QXmlStreamReader>
#include <QDebug>
void readXML(const QString &xmlData) {
    QXmlStreamReader xml(xmlData);
    while (!xml.atEnd() && !xml.hasError()) {
        QXmlStreamReader::TokenType token = xml.readNext();
        switch (token) {
        case QXmlStreamReader::StartElement:
            qDebug() << "StartElement:" << xml.name();
            break;
        case QXmlStreamReader::EndElement:
            qDebug() << "EndElement:" << xml.name();
            break;
        case QXmlStreamReader::Characters:
            if (!xml.isWhitespace())
                qDebug() << "Characters:" << xml.text();
            break;
        default:
            break;
        }
    }
    if (xml.hasError()) {
        qDebug() << "Error:" << xml.errorString();
    }
}
```
### QXmlStreamWriter
`QXmlStreamWriter` 用于写入 XML 文档。它提供了一个简单的 API 来创建 XML 元素、属性和字符数据。使用 `QXmlStreamWriter` 可以轻松地生成格式良好的 XML 文档。
以下是使用 `QXmlStreamWriter` 写入 XML 文档的一个简单示例：
```cpp
#include <QXmlStreamWriter>
#include <QDebug>
#include <QTextStream>
void writeXML() {
    QString xmlData;
    QTextStream out(&xmlData);
    QXmlStreamWriter xmlWriter(out);
    xmlWriter.setAutoFormatting(true); // 使输出格式化
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("library");
    xmlWriter.writeStartElement("book");
    xmlWriter.writeAttribute("category", "Mystery");
    xmlWriter.writeTextElement("title", "The Da Vinci Code");
    xmlWriter.writeTextElement("author", "Dan Brown");
    xmlWriter.writeEndElement(); // book
    xmlWriter.writeEndElement(); // library
    xmlWriter.writeEndDocument();
    qDebug() << xmlData;
}
```
这两个类使得在 Qt 应用程序中处理 XML 数据变得非常简单。通过使用它们，可以轻松地解析和生成 XML 文档，而无需担心底层的 XML 解析细节。

## 俩种方式的优缺点
`QXmlStreamReader`/`QXmlStreamWriter` 和 `QDomDocument` 是 Qt 中处理 XML 的两种不同的方式，每种方式都有其优缺点，适用于不同的场景。
### QXmlStreamReader/QXmlStreamWriter
- **基于流**：这两个类是基于流的，意味着它们逐个处理 XML 文档的元素，而不需要将整个文档加载到内存中。这使得它们在处理大型 XML 文档时非常高效。
- **只读或只写**：`QXmlStreamReader` 是只读的，而 `QXmlStreamWriter` 是只写的。
- **轻量级**：这两个类比 `QDomDocument` 更轻量级，因为它们不会创建一个完整的文档对象模型（DOM）。
- **易于遍历**：`QXmlStreamReader` 使得遍历 XML 文档变得非常简单，因为你只需要调用 `readNext()` 直到文档结束。
- **快速**：由于它们不需要构建整个文档的内存表示，因此在解析速度上通常比 `QDomDocument` 快。
### QDomDocument
- **文档对象模型（DOM）**：`QDomDocument` 是一个实现了 DOM 的类，它将整个 XML 文档加载到内存中，并以树状结构表示。
- **读写**：`QDomDocument` 同时支持读取和修改 XML 文档。
- **易于修改**：由于 `QDomDocument` 在内存中维护了整个文档的树状结构，因此它非常适合需要频繁修改文档内容的场景。
- **API 丰富**：`QDomDocument` 提供了丰富的 API 来操作文档，包括查询、修改和删除元素和属性。
### 选择建议
- 如果你的应用程序需要遍历大型 XML 文档，或者只需要进行只读或只写的操作，那么 `QXmlStreamReader`/`QXmlStreamWriter` 可能是更好的选择，因为它们更高效且资源消耗更少。
- 如果你的应用程序需要频繁地修改 XML 文档，或者需要随机访问文档中的元素，那么 `QDomDocument` 可能更适合，因为它提供了更灵活的 API 来操作文档。

## 总结
- 知识理应共享,[源码](https://gitee.com/shan-jie6/my-case/tree/master/QT/read_write_xml)在此。
- 这俩个类使用起来还是很简单的，看个示例下来一般就会的差不多了。
