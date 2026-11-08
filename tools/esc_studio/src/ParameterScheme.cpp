#include "ParameterScheme.h"
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QSet>

bool ParameterScheme::save(const QUrl &url)
{
    if(!url.isLocalFile())return fail(QStringLiteral("请选择本地方案文件"));
    QJsonArray entries;
    for(int g=0;g<7;++g)for(int row:m_dictionary->rowsForGroup(g)) {
        const auto &e=*m_dictionary->entry(row);
        if(!e.edited && !e.known)return fail(QStringLiteral("请先读取全部参数，或补齐草稿：%1").arg(e.name));
        QByteArray validated;QString error;
        if(!e.encode(e.decode(current(e)),&validated,&error))return fail(error);
        entries.append(QJsonObject{{"name",e.name},{"index",e.index},{"subindex",e.subindex},{"type",e.type},{"value",e.decode(current(e))}});
    }
    if(entries.isEmpty())return fail(QStringLiteral("没有配置参数"));
    if(QFileInfo::exists(url.toLocalFile()))return fail(QStringLiteral("文件已存在，请使用新文件名"));
    QSaveFile file(url.toLocalFile());if(!file.open(QIODevice::WriteOnly))return fail(file.errorString());
    auto data=QJsonDocument(QJsonObject{{"format","JESC_CONFIG"},{"version",1},
        {"dictionary_sha256",QString::fromLatin1(m_dictionary->hash())},{"entries",entries}}).toJson();
    if(file.write(data)!=data.size() || !file.commit())return fail(file.errorString());
    m_message=QStringLiteral("已保存 %1 项参数方案；包含本地草稿，不自动写设备。").arg(entries.size());emit changed();return true;
}
bool ParameterScheme::preview(const QUrl &url)
{
    m_values.clear();m_differences.clear();
    if(!url.isLocalFile())return fail(QStringLiteral("请选择本地方案文件"));
    QFile file(url.toLocalFile());if(!file.open(QIODevice::ReadOnly))return fail(file.errorString());
    if(file.size()>2*1024*1024)return fail(QStringLiteral("方案文件过大"));
    QJsonParseError parse;auto doc=QJsonDocument::fromJson(file.readAll(),&parse);auto root=doc.object();
    if(parse.error!=QJsonParseError::NoError || root["format"]!="JESC_CONFIG" || root["version"].toInt()!=1 ||
       root["dictionary_sha256"].toString().toLatin1()!=m_dictionary->hash())return fail(QStringLiteral("方案格式或对象字典版本不匹配"));
    int expected=0;for(int g=0;g<7;++g)expected+=m_dictionary->rowsForGroup(g).size();
    auto entries=root["entries"].toArray();if(!expected || entries.size()!=expected)return fail(QStringLiteral("方案必须包含完整的七组配置"));
    QVector<Value> values;QVariantList differences;QSet<int> seen;
    for(const auto &value:entries) {
        auto v=value.toObject();int row=m_dictionary->find(v["name"].toString());auto e=m_dictionary->entry(row);
        if(!e || e->group()<0 || !e->subindex || e->access!="rw" || seen.contains(row) || v["index"].toInt(-1)!=e->index ||
           v["subindex"].toInt(-1)!=e->subindex || v["type"].toInt(-1)!=e->type || !v["value"].isString())return fail(QStringLiteral("方案含重复、未知或不匹配条目"));
        QByteArray bytes;QString error;if(!e->encode(v["value"].toString(),&bytes,&error))return fail(error);
        seen.insert(row);auto before=current(*e);values.append({row,v["value"].toString(),before,bytes});
        if(before!=bytes)differences.append(QVariantMap{{"name",e->name},{"before",before.size()==e->size?e->decode(before):QStringLiteral("未知")},
            {"after",e->decode(bytes)},{"unit",e->unit}});
    }
    m_values=values;m_previewHash=m_dictionary->hash();m_differences=differences;m_message=QStringLiteral("校验通过：%1 项与当前草稿/设备值不同。确认后只载入本地草稿。").arg(differences.size());emit changed();return true;
}
bool ParameterScheme::apply()
{
    if(m_values.isEmpty())return fail(QStringLiteral("请先预览方案"));
    if(m_previewHash!=m_dictionary->hash()){m_values.clear();return fail(QStringLiteral("对象字典已改变，请重新预览"));}
    for(const auto &v:m_values){auto e=m_dictionary->entry(v.row);
        if(!e || current(*e)!=v.before){m_values.clear();return fail(QStringLiteral("预览后参数发生变化，请重新预览"));}}
    for(const auto &v:m_values)m_dictionary->editValue(v.row,v.text);
    m_values.clear();m_message=QStringLiteral("方案已载入本地草稿。请显式写入、应用，持久化需另行保存 Flash。");emit changed();return true;
}
