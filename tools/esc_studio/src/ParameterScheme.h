#pragma once
#include "Dictionary.h"
#include <QUrl>

class ParameterScheme:public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariantList differences READ differences NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
    Q_PROPERTY(bool ready READ ready NOTIFY changed)
public:
    explicit ParameterScheme(Dictionary *d,QObject *parent=nullptr):QObject(parent),m_dictionary(d) {}
    QVariantList differences() const {return m_differences;}
    QString message() const {return m_message;}
    bool ready() const {return !m_values.isEmpty();}
    bool save(const QUrl &);
    bool preview(const QUrl &);
    bool apply();
signals:
    void changed();
private:
    Dictionary *m_dictionary;
    QVariantList m_differences;
    QString m_message;
    struct Value {int row;QString text;QByteArray before,bytes;};
    QVector<Value> m_values;
    QByteArray m_previewHash;
    QByteArray current(const Entry &e) const {return e.edited?e.draft:e.device;}
    bool fail(const QString &error){m_message=error;emit changed();return false;}
};
