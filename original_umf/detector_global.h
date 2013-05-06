#ifndef _UMF_DETECTOR_GLOBAL_H
#define _UMF_DETECTOR_GLOBAL_H

#include "structures.h"
#include "pixel_counter.h"

class DetectorResult
{
public:
    DetectorResult() {
        this->vanishing[0].a = 0;this->vanishing[0].b = 0;this->vanishing[0].c = 0;
        this->vanishing[1].a = 0;this->vanishing[1].b = 0;this->vanishing[1].c = 0;
    }
    ~DetectorResult() {}
    void setVanishing(int index, Line vanish) { vanishing[index] = vanish; }
    void swapVanishing() { std::swap(vanishing[0], vanishing[1]); std::swap(ks[0], ks[1]); }
    Line* getVanishing(){ return vanishing; }
    float* getKs(){ return ks; }

private:
    Line vanishing[2];
    float ks[2];
};

typedef Singleton<DetectorResult> SDetectorResult;


#endif // DETECTOR_GLOBAL_H
