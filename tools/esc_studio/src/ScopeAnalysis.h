#pragma once
#include "ScopeData.h"
namespace ScopeAnalysis {
QVariantMap analyze(const QVector<std::shared_ptr<const ScopeBlock>> &,int channel,double from,double to);
}
