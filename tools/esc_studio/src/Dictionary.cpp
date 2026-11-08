#include "Dictionary.h"
#include "od_wire.h"
#include <QFile>
#include <QRegularExpression>
#include <QSet>
#include <QtEndian>
#include <cmath>
#include <cstring>
#include <limits>
#include <QCryptographicHash>
#include <QVariantMap>

namespace {
struct Type { const char *name; quint16 code; quint8 size; };
const Type types[] = {{"BOOLEAN",1,1},{"INTEGER8",2,1},{"INTEGER16",3,2},{"INTEGER32",4,4},
    {"UNSIGNED8",5,1},{"UNSIGNED16",6,2},{"UNSIGNED32",7,4},{"REAL32",8,4},
    {"REAL64",0x11,8},{"INTEGER64",0x15,8},{"UNSIGNED64",0x1B,8}};

// RFC-style quoted fields, including embedded commas, CRLF and doubled quotes.
bool parseCsv(QString text, QVector<QStringList> *rows, QString *error)
{
    if (text.startsWith(QChar(0xFEFF))) text.remove(0, 1);
    QString field; QStringList row;
    bool quoted = false, closed = false;
    for (qsizetype i = 0; i <= text.size(); ++i) {
        QChar c = i == text.size() ? QChar('\n') : text[i];
        if (quoted) {
            if (i == text.size()) { *error = QStringLiteral("CSV 引号未闭合"); return false; }
            if (c == '"') {
                if (i + 1 < text.size() && text[i + 1] == '"') { field += '"'; ++i; }
                else { quoted = false; closed = true; }
            } else field += c;
        } else if (c == ',' || c == '\n' || c == '\r') {
            row.append(field); field.clear(); closed = false;
            if (c != ',') {
                if (!(row.size() == 1 && row[0].isEmpty())) rows->append(row);
                row.clear();
                if (c == '\r' && i + 1 < text.size() && text[i + 1] == '\n') ++i;
            }
        } else if (c == '"' && field.isEmpty() && !closed) quoted = true;
        else if (closed || c == '"') { *error = QStringLiteral("CSV 引号格式错误"); return false; }
        else field += c;
    }
    return true;
}
quint64 bits(const QByteArray &data)
{
    quint64 value = 0;
    for (qsizetype i = 0; i < data.size(); ++i) value |= quint64(quint8(data[i])) << (8 * i);
    return value;
}
}

QString Entry::address() const
{ return QStringLiteral("%1:%2").arg(index,4,16,QChar('0')).arg(subindex,2,16,QChar('0')).toUpper(); }
QByteArray Entry::requestHeader() const
{
    QByteArray out(6, '\0');
    auto p = reinterpret_cast<uint8_t *>(out.data());
    od_wire_put16(p, index); p[2] = subindex; od_wire_put16(p + 3, type); p[5] = size;
    return out;
}
bool Entry::encode(const QString &text, QByteArray *out, QString *error) const
{
    QString value = text.trimmed();
    bool ok = false; quint64 raw = 0;
    if (type == 8 || type == 0x11) {
        double number = value.toDouble(&ok);
        if (ok && std::isfinite(number)) {
            if (type == 8) {
                float single = float(number); quint32 word;
                ok = std::isfinite(single);
                std::memcpy(&word, &single, 4); raw = word;
            } else std::memcpy(&raw, &number, 8);
        } else ok = false;
    } else {
        bool hex = value.startsWith("0x", Qt::CaseInsensitive) || value.startsWith("-0x", Qt::CaseInsensitive)
            || value.startsWith("+0x", Qt::CaseInsensitive);
        int base = hex ? 16 : 10;
        bool signedType = type == 2 || type == 3 || type == 4 || type == 0x15;
        if (signedType) {
            qint64 integer = value.toLongLong(&ok, base);
            if (ok && size < 8) {
                qint64 bound = qint64(1) << (size * 8 - 1);
                ok = integer >= -bound && integer < bound;
            }
            raw = quint64(integer);
        } else {
            raw = value.toULongLong(&ok, base);
            if (value.startsWith('-')) ok = false;
            if (ok && size < 8) ok = raw < (quint64(1) << (size * 8));
            if (type == 1) ok = ok && raw <= 1;
        }
    }
    if (!ok) { *error = QStringLiteral("%1：%2 数值无效或超出范围").arg(name, typeName); return false; }
    *out = QByteArray(size, '\0');
    for (int i = 0; i < size; ++i) (*out)[i] = char(raw >> (8 * i));
    return true;
}
QString Entry::decode(const QByteArray &data) const
{
    if (data.size() != size) return QStringLiteral("长度错误");
    quint64 raw = bits(data);
    if (type == 8) { quint32 word = quint32(raw); float v; std::memcpy(&v,&word,4); return QString::number(v,'g',9); }
    if (type == 0x11) { double v; std::memcpy(&v,&raw,8); return QString::number(v,'g',17); }
    if (type == 2 || type == 3 || type == 4 || type == 0x15) {
        if (size < 8 && (raw & (quint64(1) << (size * 8 - 1)))) raw |= ~((quint64(1) << (size * 8)) - 1);
        qint64 signedValue; std::memcpy(&signedValue,&raw,8);
        return QString::number(signedValue);
    }
    return QString::number(raw);
}

Dictionary::Dictionary(QObject *parent) : QAbstractListModel(parent) {}
int Dictionary::rowCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : int(m_entries.size()); }
QHash<int, QByteArray> Dictionary::roleNames() const
{
    return {{EntryId,"entryId"},{Name,"name"},{Description,"description"},{Address,"address"},
        {TypeName,"typeName"},{DeviceValue,"deviceValue"},{DraftValue,"draftValue"},
        {Dirty,"dirty"},{Writable,"writable"},{Group,"groupId"},{State,"stateText"},
        {Unit,"unitText"},{Section,"sectionName"},{Options,"enumOptions"}};
}
QVariant Dictionary::data(const QModelIndex &index, int role) const
{
    const Entry *e = entry(index.row());
    if (!index.isValid() || !e) return {};
    switch (role) {
    case EntryId: return index.row(); case Name: return e->name;
    case Description: return e->description; case Address: return e->address();
    case TypeName: return e->typeName; case DeviceValue: return e->known ? e->decode(e->device) : QStringLiteral("—");
    case DraftValue: return e->edited ? e->draftText : (e->known ? e->decode(e->device) : QString());
    case Dirty: return e->edited; case Writable: return e->access == "rw" && e->subindex != 0;
    case Group: return e->group(); case State: return e->status;
    case Unit:return e->unit;case Section:return e->section;case Options:return e->options;default: return {};
    }
}
bool Dictionary::load(const QString &path, QString *error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) { *error = file.errorString(); return false; }
    if (file.size() > 4 * 1024 * 1024) { *error = QStringLiteral("CSV 文件过大"); return false; }
    QVector<QStringList> rows;
    auto csv=file.readAll();
    if (!parseCsv(QString::fromUtf8(csv), &rows, error)) return false;
    if (rows.size() < 2) { *error = QStringLiteral("CSV 无对象条目"); return false; }
    const QStringList header = rows.takeFirst();
    for (const QString key : {"name","index","subindex","data_type","access"})
        if (header.count(key) != 1) { *error = QStringLiteral("CSV 缺少或重复列：%1").arg(key); return false; }
    QVector<Entry> entries; QSet<QString> names; QSet<quint32> addresses;
    const QRegularExpression namePattern("^[A-Za-z][A-Za-z0-9_]*$");
    for (const QStringList &row : rows) {
        if (row.size() != header.size()) { *error = QStringLiteral("CSV 列数不匹配"); return false; }
        auto value = [&](const QString &key) { int i = header.indexOf(key); return i < 0 ? QString() : row[i]; };
        Entry e; e.name = value("name"); e.description = value("description");
        e.typeName = value("data_type"); e.access = value("access");
        bool indexOk, subOk;
        uint index = value("index").toUInt(&indexOk,16), sub = value("subindex").toUInt(&subOk,16);
        for (const Type &type : types) if (e.typeName == QLatin1String(type.name)) { e.type=type.code; e.size=type.size; }
        if (!namePattern.match(e.name).hasMatch() || names.contains(e.name) || !indexOk || !subOk ||
            !value("index").startsWith("0x",Qt::CaseInsensitive) || !value("subindex").startsWith("0x",Qt::CaseInsensitive) ||
            index > 65535 || sub > 255 || e.size == 0 ||
            (e.access != "ro" && e.access != "wo" && e.access != "rw") || addresses.contains(index * 256 + sub)) {
            *error = QStringLiteral("CSV 条目无效或重复：%1").arg(e.name); return false;
        }
        e.index = quint16(index); e.subindex = quint8(sub);
        e.section=QStringLiteral("基本参数");
        QString path=e.description.section(' ',1);
        if(path.contains('.'))e.section=path.section('.',0,0);
        if(e.type==1)e.options={QVariantMap{{"label",QStringLiteral("否 / 0")},{"value","0"}},QVariantMap{{"label",QStringLiteral("是 / 1")},{"value","1"}}};
        auto choices=[&](QStringList labels){for(int i=0;i<labels.size();++i)e.options.append(QVariantMap{{"label",labels[i]+QString(" / %1").arg(i)},{"value",QString::number(i)}});};
        if(e.name=="motor0_type")choices({"BLDC 三相","DC"});
        if(e.name=="motor0_struct" || e.name=="encoder0_struct")choices({"直线","旋转"});
        if(e.name=="encoder0_type")choices({"ABZ","TAMAGAWA","HCFA","SPI AS5048A"});
        if(e.name=="encoder0_velocity_solver_type")choices({"M 法","估算器"});
        if(e.name=="motor_controller0_motor_controller_type")choices({"FOC","DTC"});
        if(e.name=="motor_controller0_current_decoupling_type")choices({"无","RL","交叉","RL + 交叉","反电势","RL + 反电势","交叉 + 反电势","RL + 交叉 + 反电势"});
        if(e.name=="motor_observer0_observser_config_observer_type")choices({"无","NESO2","NESO3","KF","EKF"});
        if(e.name.startsWith("loop_controller0_") && e.name.endsWith("_controller_type")) {
            if(e.name.contains("force_controller"))choices({"导纳","阻抗"});else choices({"并联 PID","串联 PID"});
        }
        if(e.name.startsWith("loop_filter0_") && e.name.endsWith("_filter_type"))choices({"无","低通","高通","带通","带阻"});
        if(e.name.startsWith("loop_filter0_") && e.name.endsWith("_filter_struct"))choices({"直通","IIR","FIR"});
        if(e.name.startsWith("loop_filter0_") && (e.name.endsWith("_filter_freq") || e.name.endsWith("_filter_width")))e.unit="Hz";
        if(e.name.startsWith("loop_filter0_") && e.name.endsWith("_filter_depth"))e.unit="dB";
        if(e.name.contains("phase_current") || e.name=="motor0_rated_current" || e.name=="motor0_peak_current")e.unit="A";
        if(e.name.contains("phase_voltage") || e.name=="motor0_rated_voltage" || e.name=="power_v_bus" || e.name=="power_v_bus_f")e.unit="V";
        if(e.name.endsWith("_rs"))e.unit=QStringLiteral("Ω");
        if(e.name.endsWith("_ld") || e.name.endsWith("_lq"))e.unit="H";
        if(e.name.endsWith("_us") || e.name.endsWith("_us_max"))e.unit=QStringLiteral("μs");
        names.insert(e.name); addresses.insert(index * 256 + sub); entries.append(e);
    }
    beginResetModel(); m_entries = std::move(entries); m_source = path;
    m_hash=QCryptographicHash::hash(csv,QCryptographicHash::Sha256).toHex();endResetModel();
    emit summaryChanged(); emit sourceChanged(); return true;
}
const Entry *Dictionary::entry(int row) const { return row >= 0 && row < m_entries.size() ? &m_entries[row] : nullptr; }
int Dictionary::find(const QString &name) const
{ for (int i=0;i<m_entries.size();++i) if (m_entries[i].name == name) return i; return -1; }
QVector<int> Dictionary::rowsForGroup(int group, bool dirtyOnly) const
{
    QVector<int> result;
    for (int i=0;i<m_entries.size();++i) if (m_entries[i].group()==group && m_entries[i].subindex!=0 &&
        (!dirtyOnly || m_entries[i].edited)) result.append(i);
    return result;
}
int Dictionary::dirtyCount() const { int count=0; for (const Entry &e:m_entries) count += e.edited; return count; }
bool Dictionary::editValue(int row, const QString &text)
{
    const Entry *found = entry(row);
    if (!found || found->access != "rw" || found->group() < 0 || found->subindex == 0) return false;
    QByteArray bytes; QString error;
    if (!found->encode(text,&bytes,&error)) { emit validationError(error); return false; }
    Entry &e=m_entries[row]; e.draft=bytes; e.draftText=e.decode(bytes);
    e.edited=!e.known || e.device!=bytes;
    e.status=e.edited ? QStringLiteral("本地修改") : QStringLiteral("与设备一致");
    emit dataChanged(index(row),index(row)); emit summaryChanged(); return true;
}
void Dictionary::discard(int row)
{
    if (!entry(row)) return;
    Entry &e=m_entries[row]; e.edited=false; e.draft.clear(); e.draftText.clear();
    e.status=e.known ? QStringLiteral("已读取") : QStringLiteral("未读取");
    emit dataChanged(index(row),index(row)); emit summaryChanged();
}
void Dictionary::acceptRead(int row,const QByteArray &bytes)
{
    if (!entry(row) || bytes.size()!=m_entries[row].size) return;
    Entry &e=m_entries[row]; e.device=bytes; e.known=true;
    if (e.edited && e.draft==bytes) e.edited=false;
    e.status=e.edited ? QStringLiteral("本地值与设备不同") : QStringLiteral("已回读");
    emit dataChanged(index(row),index(row)); emit summaryChanged();
}
void Dictionary::acceptWrite(int row,const QByteArray &bytes)
{
    if (!entry(row)) return;
    m_entries[row].status=QStringLiteral("已写入 RAM，待回读");
    Q_UNUSED(bytes)
    emit dataChanged(index(row),index(row));
}
void Dictionary::fail(int row,const QString &message)
{ if (entry(row)) { m_entries[row].status=message; emit dataChanged(index(row),index(row)); } }
void Dictionary::resetDevice()
{
    for (Entry &e:m_entries) { e.known=false; e.device.clear(); e.status=e.edited ? QStringLiteral("本地修改，设备未知") : QStringLiteral("未读取"); }
    if (!m_entries.isEmpty()) emit dataChanged(index(0),index(rowCount()-1));
}
ParameterFilter::ParameterFilter(QObject *parent):QSortFilterProxyModel(parent) { setDynamicSortFilter(true); }
void ParameterFilter::setGroup(int group) { if(m_group==group)return; m_group=group;m_section.clear(); invalidateFilter();emit sectionChanged();emit groupChanged(); }
void ParameterFilter::setSection(const QString &v){if(v==m_section)return;m_section=v;invalidateFilter();emit sectionChanged();}
QStringList ParameterFilter::sections() const {
    QStringList result; if(!sourceModel())return result;
    for(int r=0;r<sourceModel()->rowCount();++r){auto i=sourceModel()->index(r,0);auto s=sourceModel()->data(i,Dictionary::Section).toString();
        if(sourceModel()->data(i,Dictionary::Group).toInt()==m_group && !result.contains(s))result.append(s);}
    return result;
}
void ParameterFilter::setSearch(const QString &text) { if(m_search==text)return; m_search=text; invalidateFilter(); emit searchChanged(); }
bool ParameterFilter::filterAcceptsRow(int row,const QModelIndex &parent) const
{
    auto i=sourceModel()->index(row,0,parent);
    return sourceModel()->data(i,Dictionary::Group).toInt()==m_group &&
        sourceModel()->data(i,Dictionary::Writable).toBool() &&
        (m_section.isEmpty() || sourceModel()->data(i,Dictionary::Section).toString()==m_section) &&
        (m_search.isEmpty() || sourceModel()->data(i,Dictionary::Name).toString().contains(m_search,Qt::CaseInsensitive) ||
         sourceModel()->data(i,Dictionary::Description).toString().contains(m_search,Qt::CaseInsensitive));
}
