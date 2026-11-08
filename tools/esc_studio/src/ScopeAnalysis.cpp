#include "ScopeAnalysis.h"
#include <QVariantList>
#include <complex>
#include <deque>
#include <cmath>
#include <algorithm>

QVariantMap ScopeAnalysis::analyze(const QVector<std::shared_ptr<const ScopeBlock>> &blocks,int channel,double from,double to)
{
    std::deque<double> samples;quint32 segment=0,hz=0,divisor=0,config=0;quint64 next=0;bool have=false;
    for(const auto &b:blocks) {
        if(b->end()<from || b->start()>to)continue;
        if(channel<0 || channel>=b->values.size()/b->count)continue;
        if(!have || b->segment!=segment || b->config!=config || b->hz!=hz || b->divisor!=divisor || b->tick!=next)samples.clear();
        have=true;segment=b->segment;config=b->config;hz=b->hz;divisor=b->divisor;next=b->tick+quint64(b->count)*b->divisor;
        int channels=int(b->values.size())/b->count;
        for(int i=0;i<b->count;++i) {
            double t=(double(b->tick)+double(i)*b->divisor)/b->hz;if(t<from || t>to)continue;
            double value=b->values[i*channels+channel];
            if(!std::isfinite(value)){samples.clear();continue;}
            samples.push_back(value);if(samples.size()>65536)samples.pop_front();
        }
    }
    if(samples.size()<32)return {{"error",QStringLiteral("当前窗口最近连续段不足 32 个有效原始样本；请扩大窗口或选择其他通道。")}};
    int n=1;while(n*2<=int(samples.size()))n*=2;
    auto begin=samples.end()-n;double sum=0,squares=0,low=*begin,high=*begin;
    for(auto i=begin;i!=samples.end();++i){sum+=*i;squares+=(*i)*(*i);low=std::min(low,*i);high=std::max(high,*i);}
    double mean=sum/n,pi=std::acos(-1.0),gain=0;
    QVector<std::complex<double>> fft(n);
    for(int i=0;i<n;++i){double w=0.5-0.5*std::cos(2*pi*i/(n-1));gain+=w;fft[i]=(*(begin+i)-mean)*w;}
    for(int i=1,j=0;i<n;++i){int bit=n>>1;for(;j&bit;bit>>=1)j^=bit;j^=bit;if(i<j)std::swap(fft[i],fft[j]);}
    for(int length=2;length<=n;length*=2) {
        auto root=std::polar(1.0,-2*pi/length);
        for(int i=0;i<n;i+=length){std::complex<double> w(1,0);for(int j=0;j<length/2;++j){auto u=fft[i+j],v=fft[i+j+length/2]*w;fft[i+j]=u+v;fft[i+j+length/2]=u-v;w*=root;}}
    }
    double sampleRate=double(hz)/divisor,df=sampleRate/n,peak=0;int peakBin=0;
    QVector<double> amplitudes(n/2+1);
    for(int i=1;i<=n/2;++i){amplitudes[i]=std::abs(fft[i])*(i==n/2?1.0:2.0)/gain;if(amplitudes[i]>peak){peak=amplitudes[i];peakBin=i;}}
    QVariantList points;points.append(QPointF(0,0));
    // Preserve each display bucket's actual maximum; peak measurement uses all bins.
    int step=std::max(1,(n/2+999)/1000);
    for(int first=1;first<=n/2;first+=step){int at=first;for(int i=first;i<std::min(n/2+1,first+step);++i)if(amplitudes[i]>amplitudes[at])at=i;points.append(QPointF(at*df,amplitudes[at]));}
    if(points.back().toPointF().x()<sampleRate/2)points.append(QPointF(sampleRate/2,amplitudes.back()));
    return {{"error",""},{"count",n},{"mean",mean},{"rms",std::sqrt(squares/n)},{"minimum",low},{"maximum",high},
        {"peakToPeak",high-low},{"peakHz",peakBin*df},{"peakAmplitude",peak},{"resolutionHz",df},{"nyquistHz",sampleRate/2},
        {"points",points},{"segment",segment}};
}
