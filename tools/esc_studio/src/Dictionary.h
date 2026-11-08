#pragma once
#include <QAbstractListModel>
#include <QSortFilterProxyModel>
#include <QVector>

struct Entry {
    QString name, description, typeName, access;
    quint16 index = 0, type = 0;
    quint8 subindex = 0, size = 0;
    QByteArray device, draft;
    QString unit, section;
    QVariantList options;
    QString draftText, status = QStringLiteral("未读取");
    bool known = false, edited = false;
    int group() const { return index >= 0x2000 && index <= 0x2006 ? index - 0x2000 : -1; }
    QString address() const;
    QByteArray requestHeader() const;
    bool encode(const QString &, QByteArray *, QString *) const;
    QString decode(const QByteArray &) const;
};
Q_DECLARE_METATYPE(Entry)

class Dictionary : public QAbstractListModel {
    Q_OBJECT
    Q_PROPERTY(int dirtyCount READ dirtyCount NOTIFY summaryChanged)
    Q_PROPERTY(QString source READ source NOTIFY sourceChanged)
public:
    enum Role { EntryId = Qt::UserRole + 1, Name, Description, Address, TypeName,
                DeviceValue, DraftValue, Dirty, Writable, Group, State, Unit, Section, Options };
    explicit Dictionary(QObject *parent = nullptr);
    int rowCount(const QModelIndex &parent = {}) const override;
    QVariant data(const QModelIndex &, int role) const override;
    QHash<int, QByteArray> roleNames() const override;
    bool load(const QString &, QString *error);
    Q_INVOKABLE bool editValue(int row, const QString &text);
    Q_INVOKABLE void discard(int row);
    int dirtyCount() const;
    QString source() const { return m_source; }
    QByteArray hash() const {return m_hash;}
    const Entry *entry(int row) const;
    int find(const QString &) const;
    QVector<int> rowsForGroup(int group, bool dirtyOnly = false) const;
    void acceptRead(int row, const QByteArray &);
    void acceptWrite(int row, const QByteArray &);
    void fail(int row, const QString &);
    void resetDevice();
signals:
    void summaryChanged();
    void sourceChanged();
    void validationError(QString message);
private:
    QVector<Entry> m_entries;
    QString m_source;
    QByteArray m_hash;
};

class ParameterFilter : public QSortFilterProxyModel {
    Q_OBJECT
    Q_PROPERTY(int group READ group WRITE setGroup NOTIFY groupChanged)
    Q_PROPERTY(QString search READ search WRITE setSearch NOTIFY searchChanged)
    Q_PROPERTY(QString section READ section WRITE setSection NOTIFY sectionChanged)
    Q_PROPERTY(QStringList sections READ sections NOTIFY groupChanged)
public:
    explicit ParameterFilter(QObject *parent = nullptr);
    int group() const { return m_group; }
    QString search() const { return m_search; }
    void setGroup(int);
    void setSearch(const QString &);
    QString section() const {return m_section;}
    void setSection(const QString &);
    QStringList sections() const;
signals:
    void groupChanged();
    void searchChanged();
    void sectionChanged();
protected:
    bool filterAcceptsRow(int, const QModelIndex &) const override;
private:
    int m_group = 0;
    QString m_search;
    QString m_section;
};
